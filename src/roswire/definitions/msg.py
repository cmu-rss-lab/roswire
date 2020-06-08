__all__ = ('Constant', 'ConstantValue', 'Field', 'MsgFormat', 'Message')

from typing import (Optional, Any, Union, Tuple, List, Dict, ClassVar,
                    Collection, Set, Iterator, Mapping, BinaryIO)
from io import BytesIO
import hashlib
import re
import os

from loguru import logger
from toposort import toposort_flatten as toposort
import attr
import dockerblade

from .base import is_builtin, Time, Duration
from .decode import is_simple
from .. import exceptions as exc

R_TYPE = r"[a-zA-Z0-9_/]+(?:\[\d*\])?"
R_NAME = r"[a-zA-Z0-9_/]+"
R_VAL = r".+"
R_COMMENT = r"(#.*)?"
R_FIELD = re.compile(f"^\s*({R_TYPE})\s+({R_NAME})\s*{R_COMMENT}$")
R_STRING_CONSTANT = re.compile("^\s*string\s+(\w+)\s*=\s*(.+)\s*$")
R_OTHER_CONSTANT = re.compile("^\s*(\w+)\s+(\w+)\s*=\s*([^\s]+).*$")
R_BLANK = re.compile(f"^\s*{R_COMMENT}$")

ConstantValue = Union[str, int, float]


@attr.s(frozen=True, str=False)
class Constant:
    typ = attr.ib(type=str)
    name = attr.ib(type=str)
    value = attr.ib(type=Union[str, int, float])

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Constant':
        return Constant(d['type'], d['name'], d['value'])

    def to_dict(self) -> Dict[str, Any]:
        return {'type': self.typ,
                'name': self.name,
                'value': self.value}

    def __str__(self) -> str:
        return f"{self.typ} {self.name}={str(self.value)}"


@attr.s(frozen=True, str=False, slots=True, auto_attribs=True)
class Field:
    typ: str
    name: str

    @property
    def is_array(self) -> bool:
        return '[' in self.typ

    @property
    def is_simple(self) -> bool:
        return not self.is_array and is_simple(self.typ)

    @property
    def length(self) -> Optional[int]:
        if not self.is_array:
            return None
        sz = self.typ.partition('[')[2].partition(']')[0]
        if sz == '':
            return None
        return int(sz)

    @property
    def base_type(self) -> str:
        return self.typ.partition('[')[0] if self.is_array else self.typ

    @property
    def base_typ(self) -> str:
        return self.base_type

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Field':
        return Field(d['type'], d['name'])

    def to_dict(self) -> Dict[str, str]:
        return {'type': self.typ,
                'name': self.name}

    def without_package_name(self) -> 'Field':
        typ = self.typ.partition('/')[2] if '/' in self.typ else self.typ
        return Field(typ, self.name)

    def __str__(self) -> str:
        return f"{self.typ} {self.name}"


@attr.s(frozen=True)
class MsgFormat:
    package: str = attr.ib()
    name: str = attr.ib()
    definition: str = attr.ib()
    fields: Tuple[Field, ...] = attr.ib(converter=tuple)
    constants: Tuple[Constant, ...] = attr.ib(converter=tuple)

    @staticmethod
    def toposort(fmts: Collection['MsgFormat']) -> List['MsgFormat']:
        fn_to_fmt: Dict[str, MsgFormat] = {fmt.fullname: fmt for fmt in fmts}
        fn_to_deps: Dict[str, Set[str]] = \
            {filename: {f.base_typ for f in fmt.fields
                        if not is_builtin(f.base_typ)}
             for filename, fmt in fn_to_fmt.items()}
        toposorted = list(toposort(fn_to_deps))
        missing_packages: Set[str] = set(toposorted) - set(fn_to_fmt)
        if missing_packages:
            logger.error("Messages are relied upon but are not present in "
                         f"the format database: {', '.join(missing_packages)}")
            missing_package_name = next(iter(missing_packages))
            raise exc.PackageNotFound(missing_package_name)
        return [fn_to_fmt[filename] for filename in toposorted]

    @staticmethod
    def from_file(package: str,
                  fn: str,
                  files: dockerblade.FileSystem
                  ) -> 'MsgFormat':
        """
        Constructs a message format from a .msg file for a given package.

        Parameters:
            package: the name of the package that provides the file.
            fn: the path to the .msg file.
            files: a proxy for accessing the filesystem.

        Raises:
            FileNotFoundError: if the given file cannot be found.
        """
        assert fn.endswith('.msg'), 'message format files must end in .msg'
        name: str = os.path.basename(fn[:-4])
        contents: str = files.read(fn)
        return MsgFormat.from_string(package, name, contents)

    @staticmethod
    def from_string(package: str, name: str, text: str) -> 'MsgFormat':
        """
        Constructs a message format from its description.

        Raises:
            ParsingError: if the description cannot be parsed.
        """
        typ: str
        name_const: str
        fields: List[Field] = []
        constants: List[Constant] = []

        for line in text.split('\n'):
            m_blank = R_BLANK.match(line)
            m_string_constant = R_STRING_CONSTANT.match(line)
            m_other_constant = R_OTHER_CONSTANT.match(line)
            m_field = R_FIELD.match(line)

            if m_blank:
                continue
            elif m_string_constant:
                name_const, val = m_string_constant.group(1, 2)
                constant = Constant('string', name_const, val)
                constants.append(constant)
            elif m_other_constant:
                typ, name_const, val_str = m_other_constant.group(1, 2, 3)
                val = val_str  # FIXME convert value
                constant = Constant(typ, name_const, val)
                constants.append(constant)
            elif m_field:
                typ, name_field = m_field.group(1, 2)

                # resolve the type of the field
                typ_resolved = typ
                base_typ = typ.partition('[')[0]
                if typ == 'Header':
                    typ_resolved = 'std_msgs/Header'
                elif '/' not in typ and not is_builtin(base_typ):
                    typ_resolved = f'{package}/{typ}'

                if typ != typ_resolved:
                    logger.debug(f"resolved type [{typ}]: {typ_resolved}")
                    typ = typ_resolved

                field: Field = Field(typ, name_field)
                fields.append(field)
            else:
                raise exc.ParsingError(f"failed to parse line: {line}")

        return MsgFormat(package, name, text, fields, constants)  # type: ignore  # noqa

    @staticmethod
    def from_dict(d: Dict[str, Any],
                  *,
                  package: Optional[str] = None,
                  name: Optional[str] = None
                  ) -> 'MsgFormat':
        if not package:
            package = d['package']
        if not name:
            name = d['name']
        definition = d['definition']
        fields = [Field.from_dict(dd) for dd in d.get('fields', [])]
        constants = [Constant.from_dict(dd) for dd in d.get('constants', [])]
        return MsgFormat(package, name, definition, fields, constants)  # type: ignore  # noqa

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {'package': self.package,
                             'name': self.name,
                             'definition': self.definition}
        if self.fields:
            d['fields'] = [f.to_dict() for f in self.fields]
        if self.constants:
            d['constants'] = [c.to_dict() for c in self.constants]
        return d

    @property
    def fullname(self) -> str:
        return f"{self.package}/{self.name}"

    def flatten(self,
                name_to_format: Mapping[str, 'MsgFormat'],
                ctx: Tuple[str, ...] = tuple()
                ) -> Iterator[Tuple[Tuple[str, ...], Field]]:
        for field in self.fields:
            if field.is_array or is_builtin(field.typ):
                yield (ctx, field)
            else:
                fmt = name_to_format[field.typ]
                yield from fmt.flatten(name_to_format, ctx + (field.name,))

    def md5text(self, name_to_msg: Mapping[str, 'MsgFormat']) -> str:
        """Computes the MD5 text for this format."""
        lines: List[str] = []
        lines += [str(c) for c in self.constants]
        for f in self.fields:
            if is_builtin(f.base_type):
                lines += [str(f.without_package_name())]
            else:
                f_md5 = name_to_msg[f.base_type].md5sum(name_to_msg)
                lines += [f'{f_md5} {f.name}']
        return '\n'.join(lines)

    def md5sum(self, name_to_msg: Mapping[str, 'MsgFormat']) -> str:
        """Computes the MD5 sum for this format."""
        logger.debug(f"generating md5sum: {self.fullname}")
        txt = self.md5text(name_to_msg)
        logger.debug(f"generated md5 text [{self.fullname}]:\n{txt}")
        md5sum = hashlib.md5(txt.encode('utf-8')).hexdigest()
        logger.debug(f"generated md5sum [{self.fullname}]: {md5sum}")
        return md5sum


class Message:
    """Each ROS message type has its own class that is dynamically generated
    by ROSWire at runtime. This is the base class that is used by all of those
    messages."""
    format: ClassVar[MsgFormat]

    @staticmethod
    def _to_dict_value(val: Any) -> Any:
        typ = type(val)

        if typ in (Time, Duration) or issubclass(typ, Message):
            return val.to_dict()

        if typ in (list, tuple):
            if not val:
                return []
            typ_item = type(val[0])
            if typ_item == Time or issubclass(typ_item, Message):
                return [vv.to_dict() for vv in val]
            # includes (str, int, float)
            return list(val)

        # includes (str, int, float)
        return val

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {}
        for field in self.format.fields:
            name: str = field.name
            val = getattr(self, field.name)
            d[name] = self._to_dict_value(val)
        return d

    @classmethod
    def md5sum(cls) -> str:
        """Returns the md5sum for this message type."""
        raise NotImplementedError

    @classmethod
    def read(cls, b: BinaryIO) -> 'Message':
        """Reads a binary encoding of this message from a given stream."""
        raise NotImplementedError

    @classmethod
    def decode(cls, b: bytes) -> 'Message':
        """Decodes a binary encoding of this message."""
        return cls.read(BytesIO(b))

    def write(self, b: BinaryIO) -> None:
        """Writes a binary encoding of this message to a given stream."""
        raise NotImplementedError

    def encode(self) -> bytes:
        """Returns a binary encoding of this message."""
        b = BytesIO()
        self.write(b)
        return b.getvalue()
