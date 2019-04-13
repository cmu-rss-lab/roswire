__all__ = ('TypeDatabase',)

from typing import (Collection, Type, Mapping, Iterator, Dict, ClassVar, Any,
                    Sequence, Callable, BinaryIO, List)
from collections import OrderedDict

import attr

from .msg import MsgFormat, Field, Message
from .format import FormatDatabase
from .base import Time, get_builtin
from .decode import (is_simple,
                     read_time,
                     read_duration,
                     simple_reader,
                     string_reader,
                     complex_array_reader,
                     simple_array_reader)
from .encode import (write_time,
                     write_duration,
                     simple_writer,
                     string_writer,
                     complex_array_writer,
                     simple_array_writer)


class TypeDatabase(Mapping[str, Type[Message]]):
    @classmethod
    def build(cls, db_format: FormatDatabase) -> 'TypeDatabase':
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
            ns['read'] = classmethod(cls._build_read(name_to_type, fmt))
            ns['write'] = cls._build_write(name_to_type, fmt)
            md5 = fmt.md5sum(db_format.messages)
            ns['md5sum'] = classmethod(lambda cls, md5=md5: md5)
            t: Type[Message] = type(fmt.name, (Message,), ns)
            t = attr.s(t, frozen=True, slots=True)
            name_to_type[fmt.fullname] = t
        return TypeDatabase(name_to_type.values())

    @classmethod
    def _build_read(cls,
                    name_to_type: Mapping[str, Type[Message]],
                    fmt: MsgFormat
                    ) -> Callable[[Type[Message], BinaryIO], Message]:
        """Builds a reader for a given message format."""
        def get_factory(field: Field) -> Callable[[BinaryIO], Any]:
            if field.is_simple:
                return simple_reader(field.typ)
            if field.typ == 'time':
                return read_time
            if field.typ == 'duration':
                return read_duration
            if field.typ == 'string':
                return string_reader(field.length)
            if field.is_array and is_simple(field.base_type):
                return simple_array_reader(field.base_type, field.length)
            if field.is_array and not is_simple(field.base_type):
                entry_factory: Callable[[BinaryIO], Any]
                if field.base_type == 'time':
                    entry_factory = read_time
                elif field.base_type == 'duration':
                    entry_factory = read_duration
                # FIXME how about arrays of fixed-length strings?
                elif field.base_type == 'string':
                    entry_factory = string_reader()
                elif field.base_type in name_to_type:
                    entry_factory = name_to_type[field.base_type].read
                else:
                    m = "unable to find factory for base type: {}"
                    m = m.format(field.base_type)
                    raise Exception(m)
                return complex_array_reader(entry_factory, field.length)
            if field.typ in name_to_type:
                return name_to_type[field.typ].read
            m = "unable to find factory for field: {field.name} [{field.typ}]"
            raise Exception(m)

        fields: OrderedDict[str, Callable[[BinaryIO], Any]] = OrderedDict()
        for field in fmt.fields:
            fields[field.name] = get_factory(field)

        def reader(cls: Type[Message], b: BinaryIO) -> Message:
            values: Dict[str, Any] = {}
            for name, factory in fields.items():
                values[name] = factory(b)
            return cls(**values)  # type: ignore

        return reader

    @classmethod
    def _build_write(cls,
                     name_to_type: Mapping[str, Type[Message]],
                     fmt: MsgFormat
                     ) -> Callable[[Any, BinaryIO], None]:
        """Builds a write for a given message format."""
        def get_field_writer(field: Field) -> Callable[[Any, BinaryIO], None]:
            if field.is_simple:
                return simple_writer(field.typ)
            if field.typ == 'time':
                return write_time
            if field.typ == 'duration':
                return write_duration
            if field.typ == 'string':
                return string_writer(field.length)
            if field.is_array and is_simple(field.base_type):
                return simple_array_writer(field.base_type, field.length)
            if field.is_array and not is_simple(field.base_type):
                entry_writer: Callable[[Any, BinaryIO], None]
                if field.base_type == 'time':
                    entry_writer = write_time
                elif field.base_type == 'duration':
                    entry_writer = write_duration
                # FIXME how about arrays of fixed-length strings?
                elif field.base_type == 'string':
                    entry_writer = string_writer()
                elif field.base_type in name_to_type:
                    entry_writer = name_to_type[field.base_type].write
                else:
                    raise Exception(f"unable to find writer: {field.typ}")
                return complex_array_writer(entry_writer, field.length)
            if field.typ in name_to_type:
                return name_to_type[field.typ].write
            m = "unable to find writer for field: {field.name} [{field.typ}]"
            raise Exception(m)

        field_writers: OrderedDict[str, Callable[[Any, BinaryIO], None]] = \
            OrderedDict()
        for field in fmt.fields:
            field_writers[field.name] = get_field_writer(field)

        def writer(self: Message, b: BinaryIO) -> None:
            for name, field_writer in field_writers.items():
                field_writer(getattr(self, name), b)

        return writer

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
