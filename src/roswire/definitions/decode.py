# -*- coding: utf-8 -*-
"""
This module provides code for decoding and deserialising binary ROS messages
into Python data structures.
"""
from typing import Optional, Iterator, Callable, Any
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


def is_simple(typ: str) -> bool:
    """Determines whether a given type is a simple type."""
    return typ in _SIMPLE_TO_STRUCT


def get_pattern(typ: str) -> str:
    """Returns the struct pattern for a given simple type."""
    return _SIMPLE_TO_STRUCT[typ]


def decoder_simple(typ: str) -> Callable[[bytes], Any]:
    """Returns a decoder for a specified simple type."""
    pattern = '<' + get_pattern(typ)

    def decoder(v: bytes) -> Any:
        return struct.unpack(pattern, v)[0]

    def bool_decoder(v: bytes) -> bool:
        return bool(struct.unpack(pattern, v)[0])

    return bool_decoder if typ == 'bool' else decoder


def reader_simple(typ: str) -> Callable[[BytesIO], Any]:
    """Returns a reader for a specified simple type."""
    pattern = '<' + get_pattern(typ)
    decoder = decoder_simple(typ)
    size = struct.calcsize(pattern)

    def reader(b: BytesIO) -> Any:
        return decoder(b.read(size))

    return reader


decode_int8 = decoder_simple('int8')
decode_uint8 = decoder_simple('uint8')
decode_int16 = decoder_simple('int16')
decode_uint16 = decoder_simple('uint16')
decode_int32 = decoder_simple('int32')
decode_uint32 = decoder_simple('uint32')
decode_int64 = decoder_simple('int64')
decode_uint64 = decoder_simple('uint64')
decode_float32 = decoder_simple('float32')
decode_float64 = decoder_simple('float64')
decode_char = decoder_simple('char')
decode_byte = decoder_simple('byte')
decode_bool = decoder_simple('bool')

read_int8 = reader_simple('int8')
read_uint8 = reader_simple('uint8')
read_int16 = reader_simple('int16')
read_uint16 = reader_simple('uint16')
read_int32 = reader_simple('int32')
read_uint32 = reader_simple('uint32')
read_int64 = reader_simple('int64')
read_uint64 = reader_simple('uint64')
read_float32 = reader_simple('float32')
read_float64 = reader_simple('float64')
read_char = reader_simple('char')
read_byte = reader_simple('byte')
read_bool = reader_simple('bool')
