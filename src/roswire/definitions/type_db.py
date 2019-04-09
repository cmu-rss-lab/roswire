__all__ = ('TypeDatabase', 'Message')

from typing import Collection, Type, Mapping, Iterator, Dict, ClassVar, Any

import attr

from .msg import MsgFormat, Field
from .format import FormatDatabase
from .base import Time


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
        formats = list(db_format.messages.values())
        formats = MsgFormat.toposort(formats)
        name_to_type: Dict[str, Type[Message]] = {}
        for fmt in formats:
            # FIXME find type
            ns: Dict[str, Any] = {f.name: attr.ib() for f in fmt.fields}
            ns['format'] = fmt
            t: Type[Message] = type(fmt.name, (Message,), ns)
            t = attr.s(t, frozen=True, slots=True)
            name_to_type[fmt.fullname] = t
        return TypeDatabase(name_to_type.values())

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
