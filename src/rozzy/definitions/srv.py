__all__ = ['SrvFormat']

from typing import Optional, List

import attr

from .msg import MsgFormat
from ..proxy import FileProxy
from .. import exceptions


@attr.s(frozen=True)
class SrvFormat:
    package = attr.ib(type=str)
    name = attr.ib(type=str)
    request = attr.ib(type=MsgFormat)
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
        req: MsgFormat
        res: Optional[MsgFormat]
        name_req = f"{name}Request"
        name_res = f"{name}Response"

        sections: List[str] = [ss.strip() for ss in s.split('\n---')]
        try:
            s_req, s_res = sections
        except ValueError:
            m = "bad service description: missing separator (---)"
            raise exceptions.ParsingError(m)

        req = MsgFormat.from_string(package, name_req, s_req)
        if s_res:
            res = MsgFormat.from_string(package, name_res, s_res)
        else:
            res = None

        return SrvFormat(package, name, req, res)  # type: ignore
