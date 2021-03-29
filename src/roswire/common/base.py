# -*- coding: utf-8 -*-
__all__ = ("Time", "Duration", "is_builtin", "get_builtin")

from typing import Any, Dict, Type

import attr


@attr.s(frozen=True, slots=True)
class Time:
    secs: int = attr.ib()
    nsecs: int = attr.ib()

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Time":
        return Time(d["secs"], d["nsecs"])

    def to_dict(self) -> Dict[str, int]:
        return {"secs": self.secs, "nsecs": self.nsecs}


@attr.s(frozen=True, slots=True)
class Duration:
    secs: int = attr.ib()
    nsecs: int = attr.ib()

    @staticmethod
    def between(start: Time, stop: Time) -> "Duration":
        """Computes the length of time between two timestamps."""
        assert stop.secs > start.secs or stop.nsecs >= start.nsecs
        if start.secs == stop.secs:
            return Duration(0, stop.nsecs - start.nsecs)

        d_secs = stop.secs - start.secs
        if stop.nsecs >= start.nsecs:
            return Duration(d_secs, stop.nsecs - start.nsecs)
        else:
            sec_to_nsec = 1000000000  # 1 sec = 10^9 nsecs
            return Duration(d_secs - 1, stop.nsecs + sec_to_nsec - start.nsecs)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Duration":
        return Duration(d["secs"], d["nsecs"])

    def to_dict(self) -> Dict[str, int]:
        return {"secs": self.secs, "nsecs": self.nsecs}


_BUILTIN_TYPES: Dict[str, Type] = {
    "bool": bool,
    "int8": int,
    "uint8": int,
    "int16": int,
    "uint16": int,
    "int32": int,
    "uint32": int,
    "int64": int,
    "uint64": int,
    "float32": float,
    "float64": float,
    "string": str,
    "wstring": str,
    "time": Time,
    "duration": Duration,
    "char": int,  # deprecated: alias for uint8
    "byte": int,  # deprecated: alias for int8
}


def is_builtin(typ: str) -> bool:
    """Determines whether a given type is a built-in type."""
    return typ in _BUILTIN_TYPES


def get_builtin(typ: str) -> Type:
    """Returns the Python type that implements a given ROS type."""
    return _BUILTIN_TYPES[typ]
