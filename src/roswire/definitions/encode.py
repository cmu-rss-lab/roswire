# -*- coding: utf-8 -*-
"""
This module provides code for encoding and serialising Python data structures
into their corresponding ROS binary representations.
"""
from typing import (Optional, Iterator, Callable, Any, List, Type, TypeVar,
                    BinaryIO)
import functools
import struct

from .base import Time, Duration
from .decode import is_simple, get_pattern


def simple_encoder(typ: str) -> Callable[[Any], bytes]:
    """Returns an encoder for a specified simple type."""
    pattern = '<' + get_pattern(typ)
    return functools.partial(struct.pack, pattern)


def simple_writer(typ: str) -> Callable[[BinaryIO, Any], None]:
    """Returns a writer for a specified simple type."""
    encoder = simple_encoder(typ)

    def write(b: BinaryIO, v: Any) -> None:
        b.write(encoder(v))

    return write

encode_int8 = simple_encoder('int8')
encode_uint8 = simple_encoder('uint8')
encode_int16 = simple_encoder('int16')
encode_uint16 = simple_encoder('uint16')
encode_int32 = simple_encoder('int32')
encode_uint32 = simple_encoder('uint32')
encode_int64 = simple_encoder('int64')
encode_uint64 = simple_encoder('uint64')
encode_float32 = simple_encoder('float32')
encode_float64 = simple_encoder('float64')
encode_char = simple_encoder('char')
encode_byte = simple_encoder('byte')
encode_bool = simple_encoder('bool')

write_int8 = simple_writer('int8')
write_uint8 = simple_writer('uint8')
write_int16 = simple_writer('int16')
write_uint16 = simple_writer('uint16')
write_int32 = simple_writer('int32')
write_uint32 = simple_writer('uint32')
write_int64 = simple_writer('int64')
write_uint64 = simple_writer('uint64')
write_float32 = simple_writer('float32')
write_float64 = simple_writer('float64')
write_char = simple_writer('char')
write_byte = simple_writer('byte')
write_bool = simple_writer('bool')


def encode_time(time: Time) -> bytes:
    return encode_uint32(time.secs) + encode_uint32(time.nsecs)
