__all__ = ('Time',)

from typing import Dict, Any

import attr


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
