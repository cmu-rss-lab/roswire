# -*- coding: utf-8 -*-
__all__ = ("ROS2FormatDatabase",)

from typing import Any, Dict, Set

from ..common import (
    ActionFormat,
    FormatDatabase,
    MsgFormat,
    SrvFormat,
)


class ROS2FormatDatabase(FormatDatabase[MsgFormat, SrvFormat, ActionFormat]):

    @classmethod
    def build(cls,
              messages: Set[MsgFormat],
              services: Set[SrvFormat],
              actions: Set[ActionFormat]
              ) -> "FormatDatabase":
        return ROS2FormatDatabase(messages, services, actions)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ActionFormat.from_dict(dd) for dd in d["actions"]}
        return ROS2FormatDatabase(msg, srv, action)
