__all__ = ['ActionFormat']

from typing import Optional, List

import attr

from .msg import MsgFormat
from .. import exceptions


@attr.s(frozen=True)
class ActionFormat:
    """
    Reference: http://wiki.ros.org/actionlib
    """
    package = attr.ib(type=str)
    name = attr.ib(type=str)
    goal = attr.ib(type=MsgFormat)
    feedback = attr.ib(type=Optional[MsgFormat])
    result = attr.ib(type=Optional[MsgFormat])

    @staticmethod
    def from_string(package: str, name: str, s: str) -> 'ActionFormat':
        """
        Constructs an action format from its description.

        Raises:
            ParsingError: if the description cannot be parsed.
        """
        name_goal = f"{name}Goal"
        name_feed = f"{name}Feedback"
        name_res = f"{name}Result"

        sections: List[str] = [ss.strip() for ss in s.split('\n---')]
        try:
            s_goal, s_feed, s_res = sections
        except IndexError:
            m = "failed to parse action description: expected three sections."
            raise exceptions.ParsingError(m)

        goal = MsgFormat.from_string(package, name_goal, s_goal)

        if s_feed:
            feed = MsgFormat.from_string(package, name_feed, s_feed)
        else:
            feed = None

        if s_res:
            res = MsgFormat.from_string(package, name_res, s_res)
        else:
            res = None

        return ActionFormat(package, name, goal, feed, res)
