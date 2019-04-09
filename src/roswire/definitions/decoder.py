# -*- coding: utf-8 -*-
"""
TODO:
    * handle special types: Time and Duration
    * handle special string optimisations
"""
from typing import (Dict, Callable, Mapping, Any, Optional, List, Type)
from io import BytesIO
import struct

from .base import Time, decode_uint32
from .msg import Message, MsgFormat, Field

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


def read_uint32(b: BytesIO) -> int:
    return decode_uint32(b.read(4))


def is_simple(typ: str) -> bool:
    """Determines whether a given type is simple."""
    return typ in SIMPLE_TYPE_TO_STRUCT.keys()


def simple(typ: str) -> Callable[[BytesIO], Any]:
    pattern = SIMPLE_TYPE_TO_STRUCT[typ]
    size = struct.calcsize(f'<{pattern}')
    def decode(bfr: BytesIO) -> Any:
        return struct.unpack(pattern, bfr.read(size))

    def decode_bool(bfr: BytesIO) -> bool:
        return bool(decode(bfr))

    return decode_bool if typ == 'bool' else decode


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


def string(size: Optional[int] = None) -> Callable[[BytesIO], str]:
    def read_fixed(b: BytesIO) -> str:
        return b.read(size).decode('utf-8')

    def read_variable(b: BytesIO) -> str:
        size = read_uint32(bfr)
        return b.read(size).decode('utf-8')

    return read_fixed if size else read_variable


def field(name_to_type: Mapping[str, Type[Message]],
          field: Field
          ) -> Callable[[BytesIO], Any]:
    if field.is_array:
        if is_simple(field.base_typ):
            return simple_array(field.base_typ, field.length)
        else:
            factory_base = name_to_type[field.base_typ].decode
            return complex_array(factory_base, field.length)
    if is_simple(field.typ):
        return simple(field.typ)
    if field.typ == 'time':
        return Time.decode
    if field.typ == 'string':
        return string(field.length)
    # FIXME time and duration
    return name_to_type[field.typ].decode


def message(name_to_type: Mapping[str, Type[Message]],
            fmt: MsgFormat
            ) -> Callable[[BytesIO], Message]:
    fields: List[Tuple[str, Callable[[BytesIO], Any]]] = \
        [(f.name, field(name_to_type, f)) for f in fmt.fields]

    def decode(factory: Type[Message], bfr: BytesIO) -> Message:
        return factory(**{n: f(bfr) for n, f in fields})

    return decode
