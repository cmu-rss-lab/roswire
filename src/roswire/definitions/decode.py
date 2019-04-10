# -*- coding: utf-8 -*-
"""
This module provides code for decoding and deserialising binary ROS messages
into Python data structures.
"""
from typing import Optional, Iterator, Callable
from io import BytesIO
import struct

_SIMPLE_TO_STRUCT = {
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


def simple(typ: str) -> Callable[[BytesIO], Any]:
    """Returns a deserialiser for a specified simple type."""
    pattern = '<' + _SIMPLE_TO_STRUCT[typ]
    def decoder(b: BytesIO) -> Any:
        return struct.unpack(pattern)[0]

    def bool_decoder(b: BytesIO) -> bool:
        return bool(struct.unpack(pattern)[0])

    return bool_decoder if typ == 'bool' else decoder


decode_int8 = simple('int8')
decode_uint8 = simple('uint8')
decode_int16 = simple('int16')
decode_uint16 = simple('uint16')
decode_int32 = simple('int32')
decode_uint32 = simple('uint32')
decode_int64 = simple('int64')
decode_uint64 = simple('uint64')
decode_float32 = simple('float32')
decode_float64 = simple('float64')
decode_char = simple('char')
decode_byte = simple('byte')
decode_bool = simple('bool')
