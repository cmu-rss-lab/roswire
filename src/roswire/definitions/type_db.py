__all__ = ('TypeDatabase', 'Message')

from typing import (Collection, Type, Mapping, Iterator, Dict, ClassVar, Any,
                    Callable, List)

import attr

from .msg import MsgFormat, Field
from .format import FormatDatabase
from .base import Time


SIMPLE_TYPE_TO_STRUCT = {
    'int8': 'b',
    'uint8': 'B',
    'bool': 'B',
    'int16': 'h',
    'uint16': 'H',
    'int32': 'i',
    'uint32': 'I',
    'int64': 'q',
    'uint64': 'Q',
    'float32': 'f',
    'float64': 'd',
    # deprecated types
    'char': 'B',  # unsigned
    'byte': 'b'  # signed
}


def is_simple(typ: str) -> bool:
    """Determines whether a given type is simple."""
    return typ in SIMPLE_TYPE_TO_STRUCT.keys()


def get_struct_pattern(typs: List[str]) -> str:
    pass


def simple_array(base_typ: str,
                 size: Optional[int] = None
                 ) -> Callable[[BytesIO], List[Any]]:
    # treat char and uint8 arrays as special strings

    if size is not None:
        def get_size(bfr: BytesIO) -> int:
            return size
    else:
        def get_size(bfr: BytesIO) -> int:
            return read_uint32(bfr)

    # obtain a struct pattern for the base type
    if is_simple(base_type):
        size = get_size()
        pattern = f'{size}{get_struct_pattern([base_type])}'
        num_bytes = struct.calcsize(f'<{pattern}')
        contents = unpack(pattern, bfr.read(num_bytes))

    # if boolean, transform from byte to boolean
    contents  = [bool(e) for e in contents]


    return decode


def complex_array(factory,
                  size: Optional[int] = None
                  ) -> Callable[[BytesIO], List[Any]]:
    def decode_fixed(bfr: BytesIO) -> List[Any]:
        return [factory(bfr) for i in range(size)]

    def decode_variable(bfr: BytesIO) -> List[Any]:
        s = read_uint32(bfr)
        return [factory(bfr) for i in range(s)]

    return decode_fixed if size is not None else decode_variable


def string(base_type: str, size: Optional[int] = None):
    def decode_fixed(bfr: BytesIO) -> str:
        # apparently we still read a size var in some circumstances
        return bfr.read(size).decode('utf-8')

    def decode_variable(bfr: BytesIO) -> str:
        size = read_uint32(bfr)
        if base_type in ['uint8', 'char']:
            return bfr.read(size)
        else:
            return bfr.read(size).decode('utf-8')

    return decode_fixed if size is not None else decode_variable


def complex(**fields):
    def decode(bfr: BytesIO):
        return
    return decode


class Message:
    """
    Base class used by all messages.
    """
    format: ClassVar[MsgFormat]

    @staticmethod
    def _to_dict_value(val: Any) -> Any:
        typ = type(val)

        if typ == Time or isinstance(typ, Message):
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


class TypeDatabase(Mapping[str, Type[Message]]):
    @staticmethod
    def build(db_format: FormatDatabase) -> 'TypeDatabase':
        types = [TypeDatabase.build_type(m)
                 for m in db_format.messages.values()]
        return TypeDatabase(types)

    @staticmethod
    def build_type(fmt: MsgFormat) -> Type[Message]:
        # FIXME find type
        ns: Dict[str, Any] = {f.name: attr.ib() for f in fmt.fields}
        ns['format'] = fmt
        t: Type[Message] = type(fmt.name, (Message,), ns)
        t = attr.s(t, frozen=True, slots=True)
        return t

    def __init__(self, types: Collection[Type[Message]]) -> None:
        self.__contents: Dict[str, Type[Message]] = \
            {t.format.fullname: t for t in types}

    def __len__(self) -> int:
        return len(self.__contents)

    def __iter__(self) -> Iterator[str]:
        return self.__contents.__iter__()

    def __getitem__(self, name: str) -> Type[Message]:
        return self.__contents[name]

    def to_dict(self, message: Message) -> Dict[str, Any]:
        return message.to_dict()

    def _from_dict_value(self, field: Field, val: Any) -> Any:
        if field.is_array:
            fmt_item: MsgFormat = self[field.base_type].format
            return [self.from_dict(fmt_item, dd) for dd in val]
        if field.typ == 'time':
            return Time.from_dict(val)
        # NOTE covers simple values (e.g., str, int, float, bool)
        return val

    def from_dict(self, fmt: MsgFormat, d: Dict[str, Any]) -> Message:
        typ: Type[Message] = self[fmt.fullname]
        args: Dict[str, Any] = {f.name: self._from_dict_value(f, d[f.name])
                                for f in fmt.fields}
        return typ(**args)  # type: ignore
