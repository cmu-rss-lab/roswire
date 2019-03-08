__all__ = ['Package']

from typing import Tuple

import attr

from .msg import MsgFormat
from .srv import SrvFormat
from .action import ActionFormat


@attr.s(frozen=True)
class Package:
    name = attr.ib(type=str)
    path = attr.ib(type=str)
    messages = attr.ib(type=Tuple[MsgFormat, ...])
    services = attr.ib(type=Tuple[SrvFormat, ...])
    actions = attr.ib(type=Tuple[ActionFormat, ...])
