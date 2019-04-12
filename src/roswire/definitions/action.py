__all__ = ['ActionFormat']

from typing import Optional, List, Dict, Any
import os

import attr

from .msg import MsgFormat
from ..proxy import FileProxy
from .. import exceptions


@attr.s(frozen=True)
class ActionFormat:
    """
    Reference: http://wiki.ros.org/actionlib
    """
    package = attr.ib(type=str)
    name = attr.ib(type=str)
    definition = attr.ib(type=str)
    goal = attr.ib(type=MsgFormat)
    feedback = attr.ib(type=Optional[MsgFormat])
    result = attr.ib(type=Optional[MsgFormat])

    @staticmethod
    def from_file(package: str, fn: str, files: FileProxy) -> 'ActionFormat':
        """
        Constructs a message format from a .msg file for a given package.

        Parameters:
            package: the name of the package that provides the file.
            fn: the path to the .msg file.
            files: a proxy for accessing the filesystem.

        Raises:
            FileNotFoundError: if the given file cannot be found.
        """
        assert fn.endswith('.action'), \
            'action format files must end in .action'
        name: str = os.path.basename(fn[:-7])
        contents: str = files.read(fn)
        return ActionFormat.from_string(package, name, contents)

    @staticmethod
    def from_string(package: str, name: str, s: str) -> 'ActionFormat':
        """
        Constructs an action format from its description.

        Raises:
            ParsingError: if the description cannot be parsed.
        """
        goal: MsgFormat
        feed: Optional[MsgFormat]
        res: Optional[MsgFormat]

        name_goal = f"{name}Goal"
        name_feed = f"{name}Feedback"
        name_res = f"{name}Result"

        sections: List[str] = [ss.strip() for ss in s.split('\n---')]
        try:
            s_goal, s_res, s_feed = sections
        except ValueError:
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

        return ActionFormat(package, name, s, goal, feed, res)

    @staticmethod
    def from_dict(d: Dict[str, Any],
                  *,
                  package: Optional[str] = None
                  ) -> 'ActionFormat':
        name: str = d['name']
        definition: str = d['definition']
        if package is None:
            assert d['package'] is not None
            package = d['package']

        res: Optional[MsgFormat] = None
        feed: Optional[MsgFormat] = None
        goal: MsgFormat = \
            MsgFormat.from_dict(d['goal'], package=package, name=f'{name}Goal')

        if 'result' in d:
            res = MsgFormat.from_dict(d['result'],
                                      package=package,
                                      name=f'{name}Result')
        if 'feedback' in d:
            feed = MsgFormat.from_dict(d['feedback'],
                                       package=package,
                                       name=f'{name}Feedback')

        return ActionFormat(package, name, definition, goal, feed, res)

    def to_dict(self) -> Dict[str, Any]:
        d = {'package': self.package,
             'name': self.name,
             'definition': self.definition,
             'goal': self.goal.to_dict()}
        if self.feedback:
            d['feedback'] = self.feedback.to_dict()
        if self.result:
            d['result'] = self.result.to_dict()
        return d

    @property
    def fullname(self) -> str:
        return f"{self.package}/{self.name}"
