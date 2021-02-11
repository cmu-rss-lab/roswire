# -*- coding: utf-8 -*-
__all__ = ("ActionFormat",)

import os
from abc import ABC, abstractmethod
from typing import Any, Dict, Generic, Optional, TypeVar

import attr
import dockerblade

from ..common.msg import MsgFormat

MF = TypeVar("MF", bound=MsgFormat)


@attr.s(frozen=True)
class ActionFormat(ABC, Generic[MF]):
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
    goal: MF
        The definition of the goal message for this action.
    feedback: Optional[MF]
        The definition of the optional feedback message for this action, if
        it has one.
    result: Optional[MF]
        The definition of the optional result message for this action, if
        it has one.
    """

    package: str = attr.ib()
    name: str = attr.ib()
    definition: str = attr.ib()
    goal: MF = attr.ib()
    feedback: Optional[MF] = attr.ib()
    result: Optional[MF] = attr.ib()

    @classmethod
    def from_file(
            cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ActionFormat":
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
        return cls.from_string(package, name, contents)

    @classmethod
    @abstractmethod
    def from_string(cls, package: str, name: str, s: str) -> "ActionFormat":
        """Constructs an action format from its definition (i.e., the contents
        of a .action file).

        Raises
        ------
        ParsingError
            If the description cannot be parsed.
        """
        ...

    @classmethod
    @abstractmethod
    def from_dict(
            cls, d: Dict[str, Any], *, package: Optional[str] = None
    ) -> "ActionFormat":
        ...

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "package": self.package,
            "name": self.name,
            "definition": self.definition,
            "goal": self.goal.to_dict(),
        }
        if self.feedback:
            d["feedback"] = self.feedback.to_dict()
        if self.result:
            d["result"] = self.result.to_dict()
        return d

    @property
    def fullname(self) -> str:
        """The fully qualified name of this action."""
        return f"{self.package}/{self.name}"
