__all__ = ['SrvFormat']

from typing import Optional, List

import attr

from .msg import MsgFormat


@attr.s(frozen=True)
class SrvFormat:
    package = attr.ib(type=str)
    name = attr.ib(type=str)
    request = attr.ib(type=MsgFormat)
    response = attr.ib(type=Optional[MsgFormat])

    @staticmethod
    def from_string(package: str, name: str, s: str) -> 'SrvFormat':
        name_req = f"{name}Request"
        name_res = f"{name}Response"

        sections: List[str] = [ss.strip() for ss in s.split('\n---')]
        try:
            s_res, s_req = sections
        # TODO raise ParsingError
        except IndexError:
            m = "bad service description: missing separator (---)"
            raise Exception(m)

        req = MsgFormat.from_string(package, name_req, s_req)
        if s_res:
            res = MsgFormat.from_string(package, name_res, s_res)
        else:
            res = None

        return SrvFormat(package, name, req, res)
