# -*- coding: utf-8 -*-
__all__ = ("ROS1ActionFormat",)

from typing import Any, Dict, List, Optional

import dockerblade

from .msg import ROS1MsgFormat
from .. import exceptions
from ..common import ActionFormat


class ROS1ActionFormat(ActionFormat[ROS1MsgFormat]):

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS1ActionFormat":
        fmt = super().from_file(package, filename, files)
        assert isinstance(fmt, ROS1ActionFormat)
        return fmt

    @classmethod
    def from_string(
        cls, package: str, name: str, s: str
    ) -> "ROS1ActionFormat":
        goal: MsgFormat
        feed: Optional[MsgFormat]
        res: Optional[MsgFormat]

        name_goal = f"{name}Goal"
        name_feed = f"{name}Feedback"
        name_res = f"{name}Result"

        sections: List[str] = [ss.strip() for ss in s.split("---")]
        try:
            s_goal, s_res, s_feed = sections
        except ValueError:
            m = "failed to parse action description: expected three sections."
            raise exceptions.ParsingError(m)

        goal = ROS1MsgFormat.from_string(package, name_goal, s_goal)
        feed = ROS1MsgFormat.from_string(package, name_feed, s_feed)
        res = ROS1MsgFormat.from_string(package, name_res, s_res)
        return ROS1ActionFormat(package, name, s, goal, feed, res)

    @classmethod
    def from_dict(
        cls, d: Dict[str, Any], *, package: Optional[str] = None
    ) -> "ROS1ActionFormat":
        name: str = d["name"]
        definition: str = d["definition"]
        if package is None:
            assert d["package"] is not None
            package = d["package"]

        res: Optional[ROS1MsgFormat] = None
        feed: Optional[ROS1MsgFormat] = None
        goal: ROS1MsgFormat = ROS1MsgFormat.from_dict(
            d["goal"], package=package, name=f"{name}Goal"
        )

        if "result" in d:
            res = ROS1MsgFormat.from_dict(
                d["result"], package=package, name=f"{name}Result"
            )
        if "feedback" in d:
            feed = ROS1MsgFormat.from_dict(
                d["feedback"], package=package, name=f"{name}Feedback"
            )

        return ROS1ActionFormat(package, name, definition, goal, feed, res)
