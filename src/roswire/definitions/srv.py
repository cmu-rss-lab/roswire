# -*- coding: utf-8 -*-
__all__ = ('SrvFormat',)

from typing import Optional, List, Dict, Any
import os

import attr
import dockerblade

from .msg import MsgFormat


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SrvFormat:
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
    request: Optional[MsgFormat]
        The definition of the optional request message for this service, if
        it has one.
    response: Optional[MsgFormat]
        The definition of the optional response message for this service, if
        it has one.
    """
    package: str
    name: str
    definition: str
    request: Optional[MsgFormat]
    response: Optional[MsgFormat]

    @staticmethod
    def from_file(package: str,
                  filename: str,
                  files: dockerblade.FileSystem
                  ) -> 'SrvFormat':
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
        assert filename.endswith('.srv'), \
            'service format files must end in .srv'
        name: str = os.path.basename(filename[:-4])
        contents: str = files.read(filename)
        return SrvFormat.from_string(package, name, contents)

    @staticmethod
    def from_string(package: str, name: str, s: str) -> 'SrvFormat':
        """Constructs a service format from its definition.

        Raises
        ------
        ParsingError
            If the description cannot be parsed.
        """
        req: Optional[MsgFormat] = None
        res: Optional[MsgFormat] = None
        name_req = f"{name}Request"
        name_res = f"{name}Response"

        sections: List[str] = [ss.strip() for ss in s.split('---')]
        assert len(sections) < 3
        s_req = sections[0]
        s_res = sections[1] if len(sections) > 1 else ''

        if s_req:
            req = MsgFormat.from_string(package, name_req, s_req)
        if s_res:
            res = MsgFormat.from_string(package, name_res, s_res)

        return SrvFormat(package, name, s, req, res)  # type: ignore

    @staticmethod
    def from_dict(d: Dict[str, Any],
                  *,
                  package: Optional[str] = None
                  ) -> 'SrvFormat':
        req: Optional[MsgFormat] = None
        res: Optional[MsgFormat] = None
        name: str = d['name']
        definition: str = d['definition']
        if package is None:
            assert d['package'] is not None
            package = d['package']

        if 'request' in d:
            req = MsgFormat.from_dict(d['request'],
                                      package=package,
                                      name=f'{name}Request')
        if 'response' in d:
            res = MsgFormat.from_dict(d['response'],
                                      package=package,
                                      name=f'{name}Response')

        return SrvFormat(package, name, definition, req, res)

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {'package': self.package,
                             'name': self.name,
                             'definition': self.definition}
        if self.request:
            d['request'] = self.request.to_dict()
        if self.response:
            d['response'] = self.response.to_dict()
        return d

    @property
    def fullname(self) -> str:
        """The fully qualified name of this service format."""
        return f"{self.package}/{self.name}"
