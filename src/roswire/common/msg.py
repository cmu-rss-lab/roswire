# -*- coding: utf-8 -*-
__all__ = ("Constant", "ConstantValue", "Field", "MsgFormat", "Message")

import hashlib
import os
import re
from abc import ABC, abstractmethod
from io import BytesIO
from typing import (
    Any,
    BinaryIO,
    ClassVar,
    Collection,
    Dict,
    Generic, Iterator,
    List,
    Mapping,
    Optional,
    Set,
    Tuple,
    TypeVar, Union,
)

import attr
import dockerblade
from loguru import logger
from toposort import toposort_flatten as toposort

from .base import Duration, is_builtin, Time
from .decode import is_simple
from .. import exceptions as exc


R_COMMENT = r"(#.*)?"
R_BLANK = re.compile(f"^\s*{R_COMMENT}$")

ConstantValue = Union[str, int, float]


@attr.s(frozen=True, slots=True, str=False, auto_attribs=True)
class Constant:
    """Provides an immutable definition of a constant for a message format.

    Attributes
    ----------
    typ: str
        The name of the type used by this constant.
    name: str
        The name of this constant.
    value: Union[str, int, float]
        The value of this constant.
    """

    R_STRING_CONSTANT = re.compile("^\s*string\s+(\w+)\s*=\s*(.+)\s*$")
    R_OTHER_CONSTANT = re.compile("^\s*(\w+)\s+(\w+)\s*=\s*([^\s]+).*$")

    typ: str
    name: str
    value: Union[str, int, float]

    @classmethod
    def from_string(cls, line: str) -> "Optional[Constant]":
        """
        Produce a constant from a string, checking first if it is a valid
        constant, otherwise None.

        Parameters
        ----------
        line: str
            The line of text containing the constant.

        Returns
        -------
        Optional[Constant]
            A Constant object if the line is a constant, None otherwise.
        """
        m_string_constant = cls.R_STRING_CONSTANT.match(line)
        m_other_constant = cls.R_OTHER_CONSTANT.match(line)
        if m_string_constant:
            name_const, val = m_string_constant.group(1, 2)
            constant = Constant("string", name_const, val)
            return constant
        elif m_other_constant:
            typ, name_const, val_str = m_other_constant.group(1, 2, 3)
            val = val_str  # FIXME convert value
            constant = Constant(typ, name_const, val)
            return constant
        return None

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Constant":
        return Constant(d["type"], d["name"], d["value"])

    def to_dict(self) -> Dict[str, Any]:
        return {"type": self.typ, "name": self.name, "value": self.value}

    def __str__(self) -> str:
        return f"{self.typ} {self.name}={str(self.value)}"


@attr.s(frozen=True, str=False, slots=True, auto_attribs=True)
class Field:
    """Provides an immutable description of a message field.

    Attributes
    ----------
    typ: str
        The name of the type used this field.
    name: str
        The name of this field.
    """

    R_TYPE = r"[a-zA-Z0-9_/]+(?:\[(?:<=)?\d*\])?"
    R_NAME = r"[a-zA-Z0-9_/]+"
    R_FIELD = re.compile(f"^\s*({R_TYPE})\s+({R_NAME})\s*{R_COMMENT}$")

    typ: str
    name: str

    @classmethod
    def from_string(cls, package: str, line: str) -> "Optional[Field]":
        """
        Produce a field from a string, checking first if it is a
        valid field, otherwise None.

        Parameters
        ----------
        package: str
            The name of the package that provides the field.
        line: str
            The line of text containing the field.

        Returns
        -------
        Optional[Field]
            A Field object if the line is a constant, None otherwise.
        """
        m_field = cls.R_FIELD.match(line)

        if m_field:
            typ, name_field = m_field.group(1, 2)

            # resolve the type of the field
            typ_resolved = typ
            base_typ = typ.partition("[")[0]
            if typ == "Header":
                typ_resolved = "std_msgs/Header"
            elif "/" not in typ and not is_builtin(base_typ):
                typ_resolved = f"{package}/{typ}"

            if typ != typ_resolved:
                logger.debug(f"resolved type [{typ}]: {typ_resolved}")
                typ = typ_resolved

            field: Field = Field(typ, name_field)
            return field
        return None

    @property
    def is_array(self) -> bool:
        return "[" in self.typ

    @property
    def is_simple(self) -> bool:
        return not self.is_array and is_simple(self.typ)

    @property
    def length(self) -> Optional[int]:
        if not self.is_array:
            return None
        sz = self.typ.partition("[")[2].partition("]")[0]
        if sz == "":
            return None
        elif sz.startswith("<="):
            sz = sz[2:]
        return int(sz)

    @property
    def base_type(self) -> str:
        return self.typ.partition("[")[0] if self.is_array else self.typ

    @property
    def base_typ(self) -> str:
        return self.base_type

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Field":
        return Field(d["type"], d["name"])

    def to_dict(self) -> Dict[str, str]:
        return {"type": self.typ, "name": self.name}

    def without_package_name(self) -> "Field":
        typ = self.typ.partition("/")[2] if "/" in self.typ else self.typ
        return Field(typ, self.name)

    def __str__(self) -> str:
        return f"{self.typ} {self.name}"


F = TypeVar("F", bound=Field)


@attr.s(frozen=True, slots=True, auto_attribs=True)
class MsgFormat(ABC, Generic[F]):
    """Provides an immutable definition of a given ROS message format.

    Attributes
    ----------
    package: str
        The name of the package that defines this message format.
    name: str
        The unqualified name of the message format.
    definition: str
        The plaintext contents of the associated .msg file.
    fields: Sequence[Field]
        The fields that belong to this message format.
    constants: Sequence[Constant]
        The named constants that belong to this message format.

    References
    ----------
    * http://wiki.ros.org/msg
    """

    package: str
    name: str
    definition: str
    fields: Tuple[F, ...] = attr.ib(converter=tuple)
    constants: Tuple[Constant, ...] = attr.ib(converter=tuple)

    @staticmethod
    def toposort(fmts: Collection["MsgFormat"]) -> List["MsgFormat"]:
        fn_to_fmt: Dict[str, MsgFormat] = {fmt.fullname: fmt for fmt in fmts}
        fn_to_deps: Dict[str, Set[str]] = {
            filename: {
                f.base_typ for f in fmt.fields if not is_builtin(f.base_typ)
            }
            for filename, fmt in fn_to_fmt.items()
        }
        toposorted = list(toposort(fn_to_deps))
        missing_packages: Set[str] = set(toposorted) - set(fn_to_fmt)
        if missing_packages:
            logger.error(
                "Messages are relied upon but are not present in "
                f"the format database: {', '.join(missing_packages)}"
            )
            missing_package_name = next(iter(missing_packages))
            raise exc.PackageNotFound(missing_package_name)
        return [fn_to_fmt[filename] for filename in toposorted]

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "MsgFormat":
        """Constructs a message format from a .msg file for a given package.

        Parameters
        ----------
        package: str
            The name of the package that provides the file.
        filename: str
            The absolute path to the .msg file inside the given filesystem.
        files: dockerblade.FileSystem
            An interface to the filesystem that hosts the .msg file.

        Raises
        ------
        FileNotFoundError
            If the given file cannot be found.
        """
        assert filename.endswith(
            ".msg"
        ), "message format files must end in .msg"
        name: str = os.path.basename(filename[:-4])
        contents: str = files.read(filename)
        return cls.from_string(package, name, contents)

    @classmethod
    def from_string(cls, package: str, name: str, text: str) -> "MsgFormat":
        """Constructs a message format from its description.

        Parameters
        ----------
        package: str
            The name of the package that provides the file.
        filename: str
            The absolute path to the .msg file inside the given filesystem.
        text: str
            The message definition itself (e.g., the contents of a .msg file).

        Raises
        ------
        ParsingError
            If the description cannot be parsed.
        """
        typ: str
        name_const: str
        fields: List[F] = []
        constants: List[Constant] = []

        for line in text.split("\n"):
            m_blank = R_BLANK.match(line)
            if m_blank:
                continue

            constant = Constant.from_string(line)
            field = cls._field_from_string(package, line)
            if constant:
                constants.append(constant)
            elif field:
                fields.append(field)
            else:
                raise exc.ParsingError(f"failed to parse line: {line}")

        return cls(package, name, text, fields, constants)  # type: ignore  # noqa

    @classmethod
    @abstractmethod
    def _field_from_string(cls, package: str, line: str) -> Optional[F]:
        ...

    @classmethod
    @abstractmethod
    def _field_from_dict(cls, dict: Dict[str, Any]) -> F:
        ...

    @classmethod
    def from_dict(
        cls,
        d: Dict[str, Any],
        *,
        package: Optional[str] = None,
        name: Optional[str] = None,
    ) -> "MsgFormat":
        if not package:
            package = d["package"]
        if not name:
            name = d["name"]
        definition = d["definition"]
        fields = [cls._field_from_dict(dd) for dd in d.get("fields", [])]
        constants = [Constant.from_dict(dd) for dd in d.get("constants", [])]
        return cls(package, name, definition, fields, constants)  # type: ignore  # noqa

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "package": self.package,
            "name": self.name,
            "definition": self.definition,
        }
        if self.fields:
            d["fields"] = [f.to_dict() for f in self.fields]
        if self.constants:
            d["constants"] = [c.to_dict() for c in self.constants]
        return d

    @property
    def fullname(self) -> str:
        """The fully qualified name of this message format."""
        return f"{self.package}/{self.name}"

    def flatten(
        self,
        name_to_format: Mapping[str, "MsgFormat"],
        ctx: Tuple[str, ...] = (),
    ) -> Iterator[Tuple[Tuple[str, ...], Field]]:
        for field in self.fields:
            if field.is_array or is_builtin(field.typ):
                yield (ctx, field)
            else:
                fmt = name_to_format[field.typ]
                yield from fmt.flatten(name_to_format, ctx + (field.name,))

    def md5text(self, name_to_msg: Mapping[str, "MsgFormat"]) -> str:
        """Computes the MD5 text for this format."""
        lines: List[str] = []
        lines += [str(c) for c in self.constants]
        for f in self.fields:
            if is_builtin(f.base_type):
                lines += [str(f.without_package_name())]
            else:
                f_md5 = name_to_msg[f.base_type].md5sum(name_to_msg)
                lines += [f"{f_md5} {f.name}"]
        return "\n".join(lines)

    def md5sum(self, name_to_msg: Mapping[str, "MsgFormat"]) -> str:
        """Computes the MD5 sum for this format."""
        logger.debug(f"generating md5sum: {self.fullname}")
        txt = self.md5text(name_to_msg)
        logger.debug(f"generated md5 text [{self.fullname}]:\n{txt}")
        md5sum = hashlib.md5(txt.encode("utf-8")).hexdigest()
        logger.debug(f"generated md5sum [{self.fullname}]: {md5sum}")
        return md5sum


class Message:
    """Each ROS message type has its own class that is dynamically generated
    by ROSWire at runtime. This is the base class that is used by all of those
    messages.

    Attributes
    ----------
    format: MsgFormat
        The format used by this message.
    """

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
    def read(cls, b: BinaryIO) -> "Message":
        """Reads a binary encoding of this message from a given stream."""
        raise NotImplementedError

    @classmethod
    def decode(cls, b: bytes) -> "Message":
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
