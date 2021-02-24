# -*- coding: utf-8 -*-
__all__ = ("ROS1FormatDatabase",)

from typing import Any, Dict

from . import ROS1SrvFormat
from ..common import (
    ActionFormat,
    FormatDatabase,
    MsgFormat,
)


class ROS1FormatDatabase(FormatDatabase[MsgFormat,
                                        ROS1SrvFormat,
                                        ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {ROS1SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ActionFormat.from_dict(dd) for dd in d["actions"]}
        return cls(msg, srv, action)
