__all__ = ['ActionFormat']

from typing import Optional, List

import attr

from .msg import MsgFormat


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
        name_goal = f"{name}Goal"
        name_feedback = f"{name}Feedback"
        name_result = f"{name}Result"

        sections: List[str] = [ss.strip() for ss in s.split('\n---')]
        try:
            s_goal, s_feedback, s_result = sections
        # TODO add ParsingError
        except IndexError:
            m = "failed to parse action description: expected three sections."
            raise Exception(m)

        goal = MsgFormat.from_string(package, name_goal, s_goal)

        if s_feedback:
            feedback = MsgFormat.from_string(package, name_feedback, s_feedback)
        else:
            feedback = None

        if s_result:
            result = MsgFormat.from_string(package, name_result, s_result)
        else:
            result = None

        return ActionFormat(package, name, goal, feedback, result)
