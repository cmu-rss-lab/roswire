__all__ = ('TypeDatabase', 'Message')

from typing import (Collection, Type, Mapping, Iterator, Dict, ClassVar, Any,
                    Callable, List, Sequence)
from io import BytesIO

import attr

from . import decoder as dec
from .msg import MsgFormat, Field, Message
from .format import FormatDatabase
from .base import Time, get_builtin


class TypeDatabase(Mapping[str, Type[Message]]):
    @staticmethod
    def build(db_format: FormatDatabase) -> 'TypeDatabase':
        formats = list(db_format.messages.values())
        formats = MsgFormat.toposort(formats)
        name_to_type: Dict[str, Type[Message]] = {}
        for fmt in formats:
            ns: Dict[str, Any] = {}
            for f in fmt.fields:
                f_base_typ: Type
                if f.base_type in name_to_type:
                    f_base_typ = name_to_type[f.base_type]
                else:
                    f_base_typ = get_builtin(f.base_type)
                ns[f.name] = attr.ib(type=f_base_typ)
            ns['format'] = fmt

            # TODO build decoder
            decoder = dec.message(name_to_type, fmt)

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
