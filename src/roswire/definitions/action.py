# -*- coding: utf-8 -*-
__all__ = ('ActionFormat',)

from typing import Optional, List, Dict, Any
import os

import attr
import dockerblade

from .msg import MsgFormat
from .. import exceptions


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ActionFormat:
    """Provides an immutable definition of a
    `ROS Action <https://www.christimperley.co.uk/roswire/>`_.

    Attributes
    ----------
    package: str
        The name of the package to which this action belongs.
    name: str
        The name of this action.
    definition: str
        The plaintext definition of this action (e.g., the contents of a
        .action file).
    goal: MsgFormat
        The definition of the goal message for this action.
    feedback: Optional[MsgFormat]
        The definition of the optional feedback message for this action, if
        it has one.
    result: Optional[MsgFormat]
        The definition of the optional result message for this action, if
        it has one.
    """
    package: str
    name: str
    definition: str
    goal: MsgFormat
    feedback: Optional[MsgFormat]
    result: Optional[MsgFormat]

    @staticmethod
    def from_file(package: str,
                  filename: str,
                  files: dockerblade.FileSystem
                  ) -> 'ActionFormat':
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
        assert filename.endswith('.action'), \
            'action format files must end in .action'
        name: str = os.path.basename(filename[:-7])
        contents: str = files.read(filename)
        return ActionFormat.from_string(package, name, contents)

    @staticmethod
    def from_string(package: str, name: str, s: str) -> 'ActionFormat':
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

        sections: List[str] = [ss.strip() for ss in s.split('---')]
        try:
            s_goal, s_res, s_feed = sections
        except ValueError:
            m = "failed to parse action description: expected three sections."
            raise exceptions.ParsingError(m)

        goal = MsgFormat.from_string(package, name_goal, s_goal)
        feed = MsgFormat.from_string(package, name_feed, s_feed)
        res = MsgFormat.from_string(package, name_res, s_res)
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
        """The fully qualified name of this action."""
        return f"{self.package}/{self.name}"
