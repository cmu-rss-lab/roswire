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
