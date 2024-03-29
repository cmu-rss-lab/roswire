# -*- coding: utf-8 -*-
__all__ = ("ROS2FormatDatabase",)

from typing import Any, Dict

from .action import ROS2ActionFormat
from .msg import ROS2MsgFormat
from .srv import ROS2SrvFormat
from ..common import (
    FormatDatabase,
)


class ROS2FormatDatabase(FormatDatabase[ROS2MsgFormat,
                                        ROS2SrvFormat,
                                        ROS2ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {ROS2MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {ROS2SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ROS2ActionFormat.from_dict(dd) for dd in d["actions"]}
        return ROS2FormatDatabase(msg, srv, action)
