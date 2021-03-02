# -*- coding: utf-8 -*-
__all__ = ("ROS2FormatDatabase",)

from typing import Any, Dict

from . import ROS2ActionFormat
from ..common import (
    FormatDatabase,
    MsgFormat,
    SrvFormat,
)



class ROS2FormatDatabase(FormatDatabase[MsgFormat,
                                        SrvFormat,
                                        ROS2ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ROS2ActionFormat.from_dict(dd) for dd in d["actions"]}
        return ROS2FormatDatabase(msg, srv, action)
