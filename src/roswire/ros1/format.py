# -*- coding: utf-8 -*-
__all__ = ("ROS1FormatDatabase",)

from typing import Any, Dict

from ..common import (
    ActionFormat,
    FormatDatabase,
    MsgFormat,
    SrvFormat,
)


class ROS1FormatDatabase(FormatDatabase[MsgFormat, SrvFormat, ActionFormat]):

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ActionFormat.from_dict(dd) for dd in d["actions"]}
        return cls(msg, srv, action)
