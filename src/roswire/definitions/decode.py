# -*- coding: utf-8 -*-
"""
This module provides code for decoding and deserialising binary ROS messages
into Python data structures.
"""
from typing import Optional, Iterator, Callable
from io import BytesIO
import struct


def __simple_decoder(pattern: str) -> Callable[[BytesIO], Any]:
    pattern = '<' + pattern
    def decoder(b: BytesIO) -> Any:
        return struct.unpack(pattern)[0]
    return decoder


decode_int8 = __simple_decoder('b')
decode_uint8 = __simple_decoder('B')
decode_int16 = __simple_decoder('h')
decode_uint16 = __simple_decoder('H')
decode_int32 = __simple_decoder('i')
decode_uint32 = __simple_decoder('I')
decode_int64 = __simple_decoder('q')
decode_uint64 = __simple_decoder('Q')
decode_float32 = __simple_decoder('f')
decode_float64 = __simple_decoder('d')
decode_char = __simple_decoder('B')
decode_byte = decode_int8


def decode_bool(v: bytes) -> bool:
    return bool(struct.unpack('<B')[0])
