__all__ = ('Time', 'is_builtin')

from typing import Dict, Any, FrozenSet

import attr

_BUILTIN_TYPES = frozenset({
    'bool',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
    'float32',
    'float64',
    'string',
    'time',
    'duration',
    'char',  # deprecated: alias for uint8
    'byte'  # deprecated: alias for int8
})


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


def is_builtin(typ: str) -> bool:
    """Determines whether a given type is a built-in type."""
    return typ in _BUILTIN_TYPS
