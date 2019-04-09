__all__ = ('Time', 'Duration', 'is_builtin', 'get_builtin')

from typing import Dict, Any, FrozenSet, Type
from io import BytesIO

import attr


def decode_uint8(v: bytes) -> int:
    return struct.unpack('<B', v)[0]


def decode_uint32(v: bytes) -> int:
    return struct.unpack('<L', v)[0]


def decode_uint64(v: bytes) -> int:
    return struct.unpack('<LL', v)[0]


def decode_str(v: bytes) -> str:
    return v.decode('utf-8')


@attr.s(frozen=True, slots=True)
class Time:
    secs: int = attr.ib()
    nsecs: int = attr.ib()

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Time':
        return Time(d['secs'], d['nsecs'])

    def decode(b: BytesIO) -> 'Time':
        secs = decode_uint32(b.read(4))
        nsecs = decode_uint32(b.read(4))
        return Time(secs, nsecs)

    def to_dict(self) -> Dict[str, int]:
        return {'secs': self.secs,
                'nsecs': self.nsecs}


def decode_time(v: bytes) -> Time:
    return Time(decode_uint32(v[0:4]), decode_uint32(v[4:8]))


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
    'duration': Time,  # FIXME add Duration
    'char': int,  # deprecated: alias for uint8
    'byte': int  # deprecated: alias for int8
}


def is_builtin(typ: str) -> bool:
    """Determines whether a given type is a built-in type."""
    return typ in _BUILTIN_TYPES


def get_builtin(typ: str) -> Type:
    """Returns the Python type that implements a given ROS type."""
    return _BUILTIN_TYPES[typ]
