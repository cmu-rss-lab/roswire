# -*- coding: utf-8 -*-
__all__ = ("SrvFormat",)

import os
from abc import ABC, abstractmethod
from typing import Any, Dict, Generic, Optional, TypeVar

import attr
import dockerblade

from .msg import MsgFormat

MF = TypeVar("MF", bound=MsgFormat)


@attr.s(frozen=True)
class SrvFormat(ABC, Generic[MF]):
    """Provides an immutable definition of a given
    `ROS service format <http://wiki.ros.org/srv>`_.

    Attributes
    ----------
    package: str
        The name of the package that defines this service format.
    name: str
        The unqualified name of the service format.
    definition: str
        The plaintext contents of the associated .srv file.
    request: Optional[MF]
        The definition of the optional request message for this service, if
        it has one.
    response: Optional[MF]
        The definition of the optional response message for this service, if
        it has one.
    """

    package: str = attr.ib()
    name: str = attr.ib()
    definition: str = attr.ib()
    request: Optional[MF] = attr.ib()
    response: Optional[MF] = attr.ib()

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "SrvFormat":
        """Constructs a service format from a .srv file for a given package.

        Parameters
        ----------
        package: str
            The name of the package that provides the file.
        filename: str
            The absolute path to the .srv file.
        files: dockerblade.FileSystem
            An interface to the filesystem that hosts the .srv file.

        Raises
        ------
        FileNotFoundError
            If the given file cannot be found.
        """
        assert filename.endswith(
            ".srv"
        ), "service format files must end in .srv"
        name: str = os.path.basename(filename[:-4])
        contents: str = files.read(filename)
        return cls.from_string(package, name, contents)

    @classmethod
    @abstractmethod
    def from_string(cls, package: str, name: str, s: str) -> "SrvFormat":
        """Constructs a service format from its definition.

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
    ) -> "SrvFormat":
        ...

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            "package": self.package,
            "name": self.name,
            "definition": self.definition,
        }
        if self.request:
            d["request"] = self.request.to_dict()
        if self.response:
            d["response"] = self.response.to_dict()
        return d

    @property
    def fullname(self) -> str:
        """The fully qualified name of this service format."""
        return f"{self.package}/{self.name}"
