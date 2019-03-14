__all__ = ('TypeDatabase', 'Message')

from typing import Collection, Type, Mapping, Iterator, Dict, ClassVar, Any

import attr

from .msg import MsgFormat
from .format import FormatDatabase


class Message:
    """
    Base class used by all messages.
    """
    format: ClassVar[MsgFormat]


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
