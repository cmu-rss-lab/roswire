__all__ = ('Constant', 'ConstantValue', 'Field', 'MsgFormat', 'Message')

from typing import (Type, Optional, Any, Union, Tuple, List, Dict, ClassVar,
                    Collection, Set, Iterator, Mapping)
from io import BytesIO
import logging
import functools
import struct
import re
import os

import attr
from toposort import toposort_flatten as toposort

from .base import (is_builtin, is_simple, Time, Duration, read_uint32,
                   read_time, read_duration, get_pattern)
from ..proxy import FileProxy
from .. import exceptions

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

R_TYPE = r"[a-zA-Z0-9_/]+(?:\[\d*\])?"
R_NAME = r"[a-zA-Z0-9_/]+"
R_VAL = r".+"
R_COMMENT = r"(#.*)?"
R_FIELD = re.compile(f"^\s*({R_TYPE})\s+({R_NAME})\s*{R_COMMENT}$")
R_CONSTANT = re.compile(f"^\s*(\w+)\s+(\w+)\s*=\s*(.+)$")
R_BLANK = re.compile(f"^\s*{R_COMMENT}$")

ConstantValue = Union[str, int, float]


@attr.s(frozen=True)
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


@attr.s(frozen=True)
class Field:
    typ: str = attr.ib()
    name: str = attr.ib()

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


@attr.s(frozen=True)
class MsgFormat:
    package: str = attr.ib()
    name: str = attr.ib()
    fields: Tuple[Field, ...] = attr.ib(converter=tuple)
    constants: Tuple[Constant, ...] = attr.ib(converter=tuple)

    @staticmethod
    def toposort(fmts: Collection['MsgFormat']) -> List['MsgFormat']:
        fn_to_fmt: Dict[str, MsgFormat] = {f.fullname: f for f in fmts}
        fn_to_deps: Dict[str, Set[str]] = \
            {fn: {f.base_typ for f in fmt.fields if not is_builtin(f.base_typ)}
             for fn, fmt in fn_to_fmt.items()}
        toposorted = list(toposort(fn_to_deps))
        return [fn_to_fmt[fn] for fn in toposorted]

    @staticmethod
    def from_file(package: str, fn: str, files: FileProxy) -> 'MsgFormat':
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
        fields: List[Field] = []
        constants: List[Constant] = []

        for line in text.split('\n'):
            m_blank = R_BLANK.match(line)
            m_constant = R_CONSTANT.match(line)
            m_field = R_FIELD.match(line)

            if m_blank:
                continue
            elif m_constant:
                typ, name_const, val_str = m_constant.group(1, 2, 3)

                # FIXME convert value
                val: ConstantValue = val_str

                constant: Constant = Constant(typ, name_const, val)
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
                    logger.debug("resolved type [%s]: %s", typ, typ_resolved)
                    typ = typ_resolved

                field: Field = Field(typ, name_field)
                fields.append(field)
            else:
                raise exceptions.ParsingError(f"failed to parse line: {line}")

        return MsgFormat(package, name, fields, constants)  # type: ignore

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
        fields = [Field.from_dict(dd) for dd in d.get('fields', [])]
        constants = [Constant.from_dict(dd) for dd in d.get('constants', [])]
        return MsgFormat(package, name, fields, constants)  # type: ignore  # noqa

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {'package': self.package,
                             'name': self.name}
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


class MessageBuffer:
    def __init__(self,
                 typ: Type['Message'],
                 name_to_type: Mapping[str, Type['Message']]
                 ) -> None:
        self.__typ = typ
        self.__name_to_type = name_to_type
        self.__contents: Dict[str, Any] = {}

    def write(self,
              ctx: Tuple[str, ...],
              field: Field,
              value: Any
              ) -> None:
        d = self.__contents
        for ancestor in ctx:
            if ancestor not in d:
                d[ancestor] = {}
            d = d[ancestor]
        d[field.name] = value

    @property
    def contents(self) -> Dict[str, Any]:
        return self.__contents

    def __build(self,
                typ: Type['Message'],
                values: Dict[str, Any]
                ) -> 'Message':
        for n, v in values.items():
            if isinstance(v, dict):
                field = next(f for f in typ.format.fields if f.name == n)
                vt = self.__name_to_type[field.typ]
                values[n] = self.__build(vt, v)
        logger.debug("building message [%s] with values: %s", typ, values)
        return typ(**values)  # type: ignore

    def build(self) -> 'Message':
        return self.__build(self.__typ, self.__contents)


class Message:
    """Base class used by all messages."""
    format: ClassVar[MsgFormat]

    @staticmethod
    def _to_dict_value(val: Any) -> Any:
        typ = type(val)

        if typ in (Time, Duration) or isinstance(typ, Message):
            return val.to_dict()

        if typ in (list, tuple):
            if not typ:
                return []
            typ_item = type(typ[0])
            if typ_item == Time or isinstance(typ_item, Message):
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
    def _decode_string(cls, length: Optional[int], b: BytesIO) -> str:
        if length is None:
            logger.debug("decoding string length")
            length = read_uint32(b)
            logger.debug("decoded string length: %d characters", length)
        else:
            logger.debug("decoding fixed-length string")
        return b.read(length).decode('utf-8')

    @classmethod
    def _decode_simple_array(cls, field: Field, b: BytesIO) -> List[Any]:
        length = read_uint32(b) if field.length is None else field.length
        pattern = f"<{length}{get_pattern(field.base_type)}"
        logger.debug("simple array pattern: %s", pattern)
        num_bytes = struct.calcsize(pattern)
        logger.debug("simple array size: %d bytes", num_bytes)
        return list(struct.unpack(pattern, b.read(num_bytes)))

    @classmethod
    def _decode_complex_array(cls,
                              name_to_type: Mapping[str, Type['Message']],
                              field: Field,
                              b: BytesIO
                              ) -> List[Any]:
        # FIXME this doesn't handle Time or Duration
        def dec():
            return name_to_type[field.base_type].decode(name_to_type, b)

        length = read_uint32(b) if field.length is None else field.length
        return [dec() for i in range(length)]

    @classmethod
    def _decode_array(cls,
                      name_to_type: Mapping[str, Type['Message']],
                      field: Field,
                      b: BytesIO
                      ) -> List[Any]:
        if is_simple(field.base_type):
            return cls._decode_simple_array(field, b)
        else:
            return cls._decode_complex_array(name_to_type, field, b)

    @classmethod
    def _decode_chunk(cls,
                      msg_buffer: MessageBuffer,
                      chunk: List[Tuple[Tuple[str, ...], Field]],
                      b: BytesIO
                      ) -> None:
        chunk_names = ['.'.join(ctx + (field.name,)) for ctx, field in chunk]
        logger.debug("decoding chunk: %s", chunk_names)

        # compute the struct pattern for the chunk
        typs = [field.typ for ctx, field in chunk]
        pattern = '<' + ''.join([get_pattern(t) for t in typs])
        num_bytes = struct.calcsize(pattern)

        # read struct into buffer
        values = struct.unpack(pattern, b.read(num_bytes))
        for value, (ctx, field) in zip(values, chunk):
            field_fullname = '.'.join(ctx + (field.name,))
            logger.debug("decoded simple field [%s]: %s",
                         field_fullname, repr(value))
            msg_buffer.write(ctx, field, value)

    @classmethod
    def _decode_complex(cls,
                        name_to_type: Mapping[str, Type['Message']],
                        msg_buffer: MessageBuffer,
                        ctx: Tuple[str, ...],
                        field: Field,
                        b: BytesIO
                        ) -> None:
        field_fullname = '.'.join(ctx + (field.name,))
        logger.debug("decoding complex field [%s]", field_fullname)

        # read the value for the field
        val: Any
        if field.typ == 'string':
            logger.debug("decoding string: %s", field)
            val = cls._decode_string(field.length, b)
        elif field.typ == 'time':
            logger.debug("decoding time: %s", field_fullname)
            val = read_time(b)
            logger.debug("decoded time [%s]: %s", field_fullname, val)
        elif field.typ == 'duration':
            logger.debug("decoding duration: %s", field_fullname)
            val = read_duration(b)
            logger.debug("decoded duration [%s]: %s", field_fullname, val)
        elif field.is_array:
            val = cls._decode_array(name_to_type, field, b)
        else:
            Exception("unexpected complex field: {field.name} [{field.typ}]")

        # write the value to the buffer
        msg_buffer.write(ctx, field, val)

    @classmethod
    def decode(cls,
               name_to_type: Mapping[str, Type['Message']],
               b: BytesIO
               ) -> 'Message':
        name_to_format = {n: t.format for n, t in name_to_type.items()}
        flattened_names = ['.'.join(c + (f.name,))
                           for c, f in cls.format.flatten(name_to_format)]
        msg_buffer = MessageBuffer(cls, name_to_type)

        logger.debug("decoding message [%s]", cls.format.fullname)
        logger.debug("message fields [%s]: %s",
                     cls.format.fullname, flattened_names)

        # individually process each:
        # - contiguous chunk of simple fields
        # - complex field
        chunk: List[Tuple[Tuple[str, ...], Field]] = []
        for ctx, field in cls.format.flatten(name_to_format):
            field_fullname = '.'.join(ctx + (field.name,))
            logger.debug("decoding field [%s]", field_fullname)
            if field.is_simple:
                logger.debug("adding field to chunk [%s]", field_fullname)
                chunk.append((ctx, field))
            else:
                if chunk:
                    cls._decode_chunk(msg_buffer, chunk, b)
                    chunk.clear()
                cls._decode_complex(name_to_type, msg_buffer, ctx, field, b)
        if chunk:
            cls._decode_chunk(msg_buffer, chunk, b)

        # construct message from message buffer
        logger.debug("message buffer: %s", msg_buffer.contents)
        return msg_buffer.build()
