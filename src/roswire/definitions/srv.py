__all__ = ['SrvFormat']

from typing import Optional, List, Dict, Any
import os

import attr

from .msg import MsgFormat, Constant, Field
from ..proxy import FileProxy
from .. import exceptions


@attr.s(frozen=True)
class SrvFormat:
    package = attr.ib(type=str)
    name = attr.ib(type=str)
    definition = attr.ib(type=str)
    request = attr.ib(type=Optional[MsgFormat])
    response = attr.ib(type=Optional[MsgFormat])

    @staticmethod
    def from_file(package: str, fn: str, files: FileProxy) -> 'SrvFormat':
        """
        Constructs a service format from a .srv file for a given package.

        Parameters:
            package: the name of the package that provides the file.
            fn: the path to the .srv file.
            files: a proxy for accessing the filesystem.

        Raises:
            FileNotFoundError: if the given file cannot be found.
        """
        assert fn.endswith('.srv'), 'service format files must end in .srv'
        name: str = os.path.basename(fn[:-4])
        contents: str = files.read(fn)
        return SrvFormat.from_string(package, name, contents)

    @staticmethod
    def from_string(package: str, name: str, s: str) -> 'SrvFormat':
        """
        Constructs a service format from its description.

        Raises:
            ParsingError: if the description cannot be parsed.
        """
        req: Optional[MsgFormat] = None
        res: Optional[MsgFormat] = None
        name_req = f"{name}Request"
        name_res = f"{name}Response"

        sections: List[str] = [ss.strip() for ss in s.split('---')]
        try:
            s_req, s_res = sections
        except ValueError:
            m = "bad service description: missing separator (---)"
            raise exceptions.ParsingError(m)

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
        return f"{self.package}/{self.name}"
