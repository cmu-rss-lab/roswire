# -*- coding: utf-8 -*-
__all__ = ("ROS2ActionFormat",)

from typing import Any, Dict, List, Optional

import dockerblade

from .msg import ROS2MsgFormat
from .. import exceptions
from ..common import ActionFormat


class ROS2ActionFormat(ActionFormat[ROS2MsgFormat]):

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS2ActionFormat":
        fmt = super().from_file(package, filename, files)
        assert isinstance(fmt, ROS2ActionFormat)
        return fmt

    @classmethod
    def from_string(
        cls, package: str, name: str, s: str
    ) -> "ROS2ActionFormat":
        goal: ROS2MsgFormat
        feed: Optional[ROS2MsgFormat]
        res: Optional[ROS2MsgFormat]

        name_goal = f"{name}Goal"
        name_feed = f"{name}Feedback"
        name_res = f"{name}Result"

        sections = ["", "", ""]
        section_index = 0
        for line in [ss.strip() for ss in s.split("\n")]:
            if line.startswith("---"):
                if section_index < 2:
                    section_index += 1
                else:
                    raise exceptions.ParsingError(f"Action parsing should only have three sections: {s}")
            else:
                sections[section_index] += f"{line}\n"
        if section_index > 2:
            raise exceptions.ParsingError(f"Action parsing should only have three sections: {s}")
        try:
            s_goal, s_res, s_feed = sections
        except ValueError:
            m = "failed to parse action description: expected three sections."
            raise exceptions.ParsingError(m)

        goal = ROS2MsgFormat.from_string(package, name_goal, s_goal)
        feed = ROS2MsgFormat.from_string(package, name_feed, s_feed)
        res = ROS2MsgFormat.from_string(package, name_res, s_res)
        return ROS2ActionFormat(package, name, s, goal, feed, res)

    @classmethod
    def from_dict(
        cls, d: Dict[str, Any], *, package: Optional[str] = None
    ) -> "ROS2ActionFormat":
        name: str = d["name"]
        definition: str = d["definition"]
        if package is None:
            assert d["package"] is not None
            package = d["package"]

        res: Optional[ROS2MsgFormat] = None
        feed: Optional[ROS2MsgFormat] = None
        goal: ROS2MsgFormat = ROS2MsgFormat.from_dict(
            d["goal"], package=package, name=f"{name}Goal"
        )

        if "result" in d:
            res = ROS2MsgFormat.from_dict(
                d["result"], package=package, name=f"{name}Result"
            )
        if "feedback" in d:
            feed = ROS2MsgFormat.from_dict(
                d["feedback"], package=package, name=f"{name}Feedback"
            )

        return ROS2ActionFormat(package, name, definition, goal, feed, res)
