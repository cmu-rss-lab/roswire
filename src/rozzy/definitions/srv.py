__all__ = ['SrvFormat']

from typing import Optional

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
        name_request = f"{name}Request"
        name_response = f"{name}Response"

        s_request, separator, s_response = s.partition('\n---\n')
        s_response = s_response.strip()
        s_request = s_request.strip()
        if not separator:
            m = "bad service description: missing separator (---)"
            raise Exception(m)

        request = MsgFormat.from_string(s_request, package, name_request)
        if s_response:
            response = MsgFormat.from_string(s_request, package, name_response)
        else:
            response = None

        return SrvFormat(package, name, request, response)
