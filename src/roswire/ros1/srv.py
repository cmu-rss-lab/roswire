# -*- coding: utf-8 -*-
__all__ = ("ROS1SrvFormat",)

from typing import Any, Dict, Optional

import dockerblade

from .msg import ROS1MsgFormat
from ..common import SrvFormat
from ..exceptions import ParsingError


class ROS1SrvFormat(SrvFormat[ROS1MsgFormat]):

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS1SrvFormat":
        sf = super().from_file(package, filename, files)
        assert isinstance(sf, ROS1SrvFormat)
        return sf

    @classmethod
    def from_string(cls, package: str, name: str, s: str) -> "ROS1SrvFormat":
        req: Optional[ROS1MsgFormat] = None
        res: Optional[ROS1MsgFormat] = None
        name_req = f"{name}Request"
        name_res = f"{name}Response"

        sections = ["", ""]
        section_index = 0  # process request first
        for line in [ss.strip() for ss in s.split['\n']]:
            if line.startswith("---"):
                if section_index == 0:
                    section_index = 1
                else:
                    raise ParsingError(f"Should only be one --- in {name} svc for {package}")
            else:
                sections[section_index] += f"{line}\n"

        s_req = sections[0]
        s_res = sections[1]

        if s_req:
            req = ROS1MsgFormat.from_string(package, name_req, s_req)
        if s_res:
            res = ROS1MsgFormat.from_string(package, name_res, s_res)

        return ROS1SrvFormat(package, name, s, req, res)

    @classmethod
    def from_dict(
        cls, d: Dict[str, Any], *, package: Optional[str] = None
    ) -> "ROS1SrvFormat":
        req: Optional[ROS1MsgFormat] = None
        res: Optional[ROS1MsgFormat] = None
        name: str = d["name"]
        definition: str = d["definition"]
        if package is None:
            assert d["package"] is not None
            package = d["package"]

        if "request" in d:
            req = ROS1MsgFormat.from_dict(
                d["request"], package=package, name=f"{name}Request"
            )
        if "response" in d:
            res = ROS1MsgFormat.from_dict(
                d["response"], package=package, name=f"{name}Response"
            )

        return ROS1SrvFormat(package, name, definition, req, res)
