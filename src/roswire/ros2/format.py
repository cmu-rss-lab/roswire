# -*- coding: utf-8 -*-
__all__ = ("ROS2FormatDatabase",)

from typing import Any, Dict

from . import ROS2SrvFormat
from ..common import (
    ActionFormat,
    FormatDatabase,
    MsgFormat,
)


class ROS2FormatDatabase(FormatDatabase[MsgFormat,
                                        ROS2SrvFormat,
                                        ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {ROS2SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ActionFormat.from_dict(dd) for dd in d["actions"]}
        return ROS2FormatDatabase(msg, srv, action)
