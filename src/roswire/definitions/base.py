__all__ = ('Time', 'Duration',
           'is_builtin', 'get_builtin', 'is_simple', 'get_pattern',
           'decode_uint32', 'read_uint32',
           'decode_time', 'read_time',
           'decode_duration', 'read_duration')

from typing import Dict, Any, FrozenSet, Type
from io import BytesIO
import struct

import attr

_SIMPLE_TYPE_TO_STRUCT = {
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


@attr.s(frozen=True, slots=True)
class Time:
    secs: int = attr.ib()
    nsecs: int = attr.ib()

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Time':
        return Time(d['secs'], d['nsecs'])

    def to_dict(self) -> Dict[str, int]:
        return {'secs': self.secs,
                'nsecs': self.nsecs}


@attr.s(frozen=True, slots=True)
class Duration:
    secs: int = attr.ib()
    nsecs: int = attr.ib()

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Duration':
        return Duration(d['secs'], d['nsecs'])

    def to_dict(self) -> Dict[str, int]:
        return {'secs': self.secs,
                'nsecs': self.nsecs}


_BUILTIN_TYPES: Dict[str, Type] = {
    'bool': bool,
    'int8': int,
    'uint8': int,
    'int16': int,
    'uint16': int,
    'int32': int,
    'uint32': int,
    'int64': int,
    'uint64': int,
    'float32': float,
    'float64': float,
    'string': str,
    'time': Time,
    'duration': Duration,
    'char': int,  # deprecated: alias for uint8
    'byte': int  # deprecated: alias for int8
}


def is_builtin(typ: str) -> bool:
    """Determines whether a given type is a built-in type."""
    return typ in _BUILTIN_TYPES


def get_builtin(typ: str) -> Type:
    """Returns the Python type that implements a given ROS type."""
    return _BUILTIN_TYPES[typ]


def get_pattern(typ: str) -> str:
    """Returns the struct pattern for a simple type."""
    return _SIMPLE_TYPE_TO_STRUCT[typ]


def is_simple(typ: str) -> bool:
    """Determines whether a given type is a simple primitive."""
    return typ in _SIMPLE_TYPE_TO_STRUCT


def decode_uint32(v: bytes) -> int:
    return struct.unpack('<B', v)[0]


def read_uint32(b: BytesIO) -> int:
    return decode_uint32(b.read(4))


def decode_time(b: bytes) -> Time:
    secs = read_uint32(b)
    nsecs = read_uint32(b)
    return Time(secs, nsecs)


def read_time(b: BytesIO) -> Time:
    return decode_time(b.read(8))


def decode_duration(b: bytes) -> Duration:
    secs = read_uint32(b)
    nsecs = read_uint32(b)
    return Duration(secs, nsecs)


def read_duration(b: BytesIO) -> Duration:
    return decode_duration(b.read(8))
