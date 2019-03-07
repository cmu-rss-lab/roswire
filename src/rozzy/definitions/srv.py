__all__ = ['SrvFormat']

from typing import Optional

import attr

from .msg import MsgFormat


@attr.s(frozen=True)
class SrvFormat:
    request = attr.ib(type=MsgFormat)
    response = attr.ib(type=Optional[MsgFormat])
