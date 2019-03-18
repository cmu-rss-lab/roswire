__all__ = ('TypeDatabase', 'Message')

from typing import Collection, Type, Mapping, Iterator, Dict, ClassVar, Any

import attr

from .msg import MsgFormat
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
