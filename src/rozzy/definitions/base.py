__all__ = ('Time',)

import attr


@attr.s(frozen=True, slots=True)
class Time:
    secs: int = attr.ib()
    nsecs: int = attr.ib()
