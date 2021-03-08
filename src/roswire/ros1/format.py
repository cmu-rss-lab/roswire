# -*- coding: utf-8 -*-
__all__ = ("ROS1FormatDatabase",)

from typing import Any, Dict

from . import ROS1ActionFormat, ROS1MsgFormat, ROS1SrvFormat
from ..common import (
    FormatDatabase,
)


class ROS1FormatDatabase(FormatDatabase[ROS1MsgFormat,
                                        ROS1SrvFormat,
                                        ROS1ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {ROS1MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {ROS1SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ROS1ActionFormat.from_dict(dd) for dd in d["actions"]}
        return cls(msg, srv, action)
