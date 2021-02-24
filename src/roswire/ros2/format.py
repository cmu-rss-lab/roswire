# -*- coding: utf-8 -*-
__all__ = ("ROS2FormatDatabase",)

from typing import Any, Dict, Set

from . import ROS2ActionFormat, ROS2MsgFormat
from ..common import (
    FormatDatabase,
    SrvFormat,
)


class ROS2FormatDatabase(FormatDatabase[ROS2MsgFormat,
                                        SrvFormat,
                                        ROS2ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg: Set[ROS2MsgFormat] = {
            ROS2MsgFormat.from_dict(dd) for dd in d["messages"]
        }
        srv = {SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ROS2ActionFormat.from_dict(dd) for dd in d["actions"]}
        return ROS2FormatDatabase(msg, srv, action)
