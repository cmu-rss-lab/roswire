# -*- coding: utf-8 -*-
__all__ = ("ROS1ActionFormat",)

import os
from typing import Any, Dict, List, Optional

import dockerblade

from .. import exceptions
from ..common import ActionFormat, MsgFormat


class ROS1ActionFormat(ActionFormat[MsgFormat]):

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS1ActionFormat":
        """Constructs an action format from a .action file for a given package.

        Parameters
        ----------
        package: str
            The name of the package that provides the file.
        filename: str
            The path to the .msg file.
        files: dockerblade.FileSystem
            An interface to the filesystem that hosts the .action file.

        Raises
        ------
        FileNotFoundError
            If the given file cannot be found.
        """
        assert filename.endswith(
            ".action"
        ), "action format files must end in .action"
        name: str = os.path.basename(filename[:-7])
        contents: str = files.read(filename)
        return ROS1ActionFormat.from_string(package, name, contents)

    @classmethod
    def from_string(
        cls, package: str, name: str, s: str
    ) -> "ROS1ActionFormat":
        """Constructs an action format from its definition (i.e., the contents
        of a .action file).

        Raises
        ------
        ParsingError
            If the description cannot be parsed.
        """
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

        goal = MsgFormat.from_string(package, name_goal, s_goal)
        feed = MsgFormat.from_string(package, name_feed, s_feed)
        res = MsgFormat.from_string(package, name_res, s_res)
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

        res: Optional[MsgFormat] = None
        feed: Optional[MsgFormat] = None
        goal: MsgFormat = MsgFormat.from_dict(
            d["goal"], package=package, name=f"{name}Goal"
        )

        if "result" in d:
            res = MsgFormat.from_dict(
                d["result"], package=package, name=f"{name}Result"
            )
        if "feedback" in d:
            feed = MsgFormat.from_dict(
                d["feedback"], package=package, name=f"{name}Feedback"
            )

        return ROS1ActionFormat(package, name, definition, goal, feed, res)

    def __init__(self,
                 package: str,
                 name: str,
                 definition: str,
                 goal: MsgFormat,
                 feedback: Optional[MsgFormat],
                 result: Optional[MsgFormat]
                 ):
        self.package = package
        self.name = name
        self.definition = definition
        self.goal = goal
        self.feeback = feedback
        self.result = result
