# -*- coding: utf-8 -*-
"""
This module provides code for encoding and serialising Python data structures
into their corresponding ROS binary representations.
"""
import functools
import struct
from typing import Any, BinaryIO, Callable, Dict, Optional, Sequence, TypeVar

from .base import Duration, Time
from .decode import get_pattern

T = TypeVar("T")


def ignore(val: Any) -> None:
    """Used to prevent returning values in lambdas."""
    return None


def simple_encoder(typ: str) -> Callable[[Any], bytes]:
    """Returns an encoder for a specified simple type."""
    pattern = "<" + get_pattern(typ)
    return functools.partial(struct.pack, pattern)


def sized_encoder(
    encoder_content: Callable[[T], bytes], get_size: Callable[[T], int]
) -> Callable[[T], bytes]:
    def encoder(val: T) -> bytes:
        bin_size = encode_uint32(get_size(val))
        bin_content = encoder_content(val)
        return bin_size + bin_content

    return encoder


def writer(encoder: Callable[[Any], bytes]) -> Callable[[Any, BinaryIO], None]:
    return lambda v, b: ignore(b.write(encoder(v)))


def simple_writer(typ: str) -> Callable[[Any, BinaryIO], None]:
    """Returns a writer for a specified simple type."""
    return writer(simple_encoder(typ))


encode_int8 = simple_encoder("int8")
encode_uint8 = simple_encoder("uint8")
encode_int16 = simple_encoder("int16")
encode_uint16 = simple_encoder("uint16")
encode_int32 = simple_encoder("int32")
encode_uint32 = simple_encoder("uint32")
encode_int64 = simple_encoder("int64")
encode_uint64 = simple_encoder("uint64")
encode_float32 = simple_encoder("float32")
encode_float64 = simple_encoder("float64")
encode_char = simple_encoder("char")
encode_byte = simple_encoder("byte")
encode_bool = simple_encoder("bool")


def encode_time(time: Time) -> bytes:
    return encode_uint32(time.secs) + encode_uint32(time.nsecs)


def encode_duration(duration: Duration) -> bytes:
    return encode_uint32(duration.secs) + encode_uint32(duration.nsecs)


write_int8 = simple_writer("int8")
write_uint8 = simple_writer("uint8")
write_int16 = simple_writer("int16")
write_uint16 = simple_writer("uint16")
write_int32 = simple_writer("int32")
write_uint32 = simple_writer("uint32")
write_int64 = simple_writer("int64")
write_uint64 = simple_writer("uint64")
write_float32 = simple_writer("float32")
write_float64 = simple_writer("float64")
write_char = simple_writer("char")
write_byte = simple_writer("byte")
write_bool = simple_writer("bool")
write_time = writer(encode_time)
write_duration = writer(encode_duration)


def write_encoded_header(contents: Dict[str, bytes], out: BinaryIO) -> None:
    # write a length of zero for now and correct that length once the
    # fields have been written
    pos_size = out.tell()
    write_uint32(0, out)
    pos_content = out.tell()

    for name, bin_value in contents.items():
        bin_name = f"{name}=".encode("utf-8")
        size_field = len(bin_name) + len(bin_value)
        write_uint32(size_field, out)
        out.write(bin_name)
        out.write(bin_value)

    # correct the length
    pos_end = out.tell()
    size_total = pos_end - pos_content
    out.seek(pos_size)
    write_uint32(size_total, out)
    out.seek(pos_end)


def string_writer(
    length: Optional[int] = None,
) -> Callable[[str, BinaryIO], None]:
    """Returns a writer for (possibly fixed-length) strings."""
    encoder: Callable[[str], bytes]
    encode_content: Callable[[str], bytes] = str.encode
    if length is None:
        encoder = sized_encoder(encode_content, str.__len__)
    else:
        encoder = encode_content
    return writer(encoder)


def simple_array_writer(
    typ: str, length: Optional[int] = None
) -> Callable[[Sequence[Any], BinaryIO], None]:
    """Returns a writer for a simple array."""
    base_pattern = get_pattern(typ)

    if length is not None:
        pattern = f"<{length}{base_pattern}"
        return lambda arr, b: ignore(struct.pack(pattern, *arr))

    def var_writer(arr: Sequence[Any], b: BinaryIO) -> None:
        length = len(arr)
        write_uint32(length, b)
        pattern = f"<{length}{base_pattern}"
        b.write(struct.pack(pattern, *arr))

    return var_writer


def complex_array_writer(
    entry_writer: Callable[[Any, BinaryIO], None], length: Optional[int] = None
) -> Callable[[Any, BinaryIO], None]:
    """Returns a writer for a complex array."""

    def write_content(arr: Sequence[Any], b: BinaryIO) -> None:
        for v in arr:
            entry_writer(v, b)

    if length is not None:
        return write_content

    def var_writer(arr: Sequence[Any], b: BinaryIO) -> None:
        length = len(arr)
        write_uint32(length, b)
        write_content(arr, b)

    return var_writer
