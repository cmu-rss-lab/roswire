# -*- coding: utf-8 -*-
"""
This module provides code for decoding and deserialising binary ROS messages
into Python data structures.
"""
from typing import (Optional, Iterator, Callable, Any, List, Type, TypeVar,
                    BinaryIO)
import functools
import struct

from .base import Time, Duration

T = TypeVar('T')

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


def simple_decoder(typ: str) -> Callable[[bytes], Any]:
    """Returns a decoder for a specified simple type."""
    pattern = '<' + get_pattern(typ)

    def decoder(v: bytes) -> Any:
        return struct.unpack(pattern, v)[0]

    def bool_decoder(v: bytes) -> bool:
        return bool(struct.unpack(pattern, v)[0])

    return bool_decoder if typ == 'bool' else decoder


def simple_reader(typ: str) -> Callable[[BinaryIO], Any]:
    """Returns a reader for a specified simple type."""
    pattern = '<' + get_pattern(typ)
    decoder = simple_decoder(typ)
    size = struct.calcsize(pattern)

    def reader(b: BinaryIO) -> Any:
        return decoder(b.read(size))

    return reader


decode_int8 = simple_decoder('int8')
decode_uint8 = simple_decoder('uint8')
decode_int16 = simple_decoder('int16')
decode_uint16 = simple_decoder('uint16')
decode_int32 = simple_decoder('int32')
decode_uint32 = simple_decoder('uint32')
decode_int64 = simple_decoder('int64')
decode_uint64 = simple_decoder('uint64')
decode_float32 = simple_decoder('float32')
decode_float64 = simple_decoder('float64')
decode_char = simple_decoder('char')
decode_byte = simple_decoder('byte')
decode_bool = simple_decoder('bool')

read_int8 = simple_reader('int8')
read_uint8 = simple_reader('uint8')
read_int16 = simple_reader('int16')
read_uint16 = simple_reader('uint16')
read_int32 = simple_reader('int32')
read_uint32 = simple_reader('uint32')
read_int64 = simple_reader('int64')
read_uint64 = simple_reader('uint64')
read_float32 = simple_reader('float32')
read_float64 = simple_reader('float64')
read_char = simple_reader('char')
read_byte = simple_reader('byte')
read_bool = simple_reader('bool')


def decode_time(b: bytes) -> Time:
    secs = decode_uint32(b[0:4])
    nsecs = decode_uint32(b[4:8])
    return Time(secs, nsecs)


def read_time(b: BinaryIO) -> Time:
    return decode_time(b.read(8))


def decode_duration(b: bytes) -> Duration:
    secs = decode_uint32(b[0:4])
    nsecs = decode_uint32(b[4:8])
    return Duration(secs, nsecs)


def read_duration(b: BinaryIO) -> Duration:
    return decode_duration(b.read(8))


def decode_string(b: bytes) -> str:
    return b.decode('utf-8')


def read_fixed_length_string(size: int, b: BinaryIO) -> str:
    """Reads a fixed-length string from a bytestream."""
    return decode_string(b.read(size))


def read_string(b: BinaryIO) -> str:
    """Reads a variable-length string from a bytestream."""
    size = read_uint32(b)
    return read_fixed_length_string(size, b)


def string_reader(length: Optional[int] = None
                  ) -> Callable[[BinaryIO], str]:
    """Returns a reader for (possibly fixed-length) strings."""
    if length is None:
        return read_string
    else:
        return functools.partial(read_fixed_length_string, length)


def simple_array_reader(typ: str,
                        length: Optional[int] = None
                        ) -> Callable[[BinaryIO], List[Any]]:
    """Returns a reader for a simple array."""
    base_pattern = get_pattern(typ)

    # fixed length: precompute pattern
    if length is not None:
        pattern = f'<{length}{base_pattern}'
        size = struct.calcsize(pattern)

        def fixed_reader(b: BinaryIO) -> List[Any]:
            return list(struct.unpack(pattern, b.read(size)))

        return fixed_reader

    # variable length
    def var_reader(b: BinaryIO) -> List[Any]:
        length = read_uint32(b)
        pattern = f'<{length}{base_pattern}'
        size = struct.calcsize(pattern)
        return list(struct.unpack(pattern, b.read(size)))

    return var_reader


def complex_array_reader(factory: Callable[[BinaryIO], T],
                         length: Optional[int] = None
                         ) -> Callable[[BinaryIO], List[T]]:
    """Returns a reader for a complex array."""
    def read_fixed(length: int, b: BinaryIO) -> List[T]:
        return [factory(b) for i in range(length)]

    def read_var(b: BinaryIO) -> List[T]:
        length = read_uint32(b)
        return read_fixed(length, b)

    if length is None:
        return read_var
    else:
        return functools.partial(read_fixed, length)
