# -*- coding: utf-8 -*-
__all__ = ("ROS2SrvFormat", )

from typing import Any, Dict, List, Optional

import dockerblade

from ..common import MsgFormat, SrvFormat


class ROS2SrvFormat(SrvFormat[MsgFormat]):

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS2SrvFormat":
        sf = super().from_file(package, filename, files)
        assert isinstance(sf, ROS2SrvFormat)
        return sf

    @classmethod
    def from_string(cls, package: str, name: str, s: str) -> "ROS2SrvFormat":
        req: Optional[MsgFormat] = None
        res: Optional[MsgFormat] = None
        name_req = f"{name}Request"
        name_res = f"{name}Response"

        sections: List[str] = [ss.strip() for ss in s.split("---")]
        assert len(sections) < 3
        s_req = sections[0]
        s_res = sections[1] if len(sections) > 1 else ""

        if s_req:
            req = MsgFormat.from_string(package, name_req, s_req)
        if s_res:
            res = MsgFormat.from_string(package, name_res, s_res)

        return ROS2SrvFormat(package, name, s, req, res)

    @classmethod
    def from_dict(
        cls, d: Dict[str, Any], *, package: Optional[str] = None
    ) -> "ROS2SrvFormat":
        req: Optional[MsgFormat] = None
        res: Optional[MsgFormat] = None
        name: str = d["name"]
        definition: str = d["definition"]
        if package is None:
            assert d["package"] is not None
            package = d["package"]

        if "request" in d:
            req = MsgFormat.from_dict(
                d["request"], package=package, name=f"{name}Request"
            )
        if "response" in d:
            res = MsgFormat.from_dict(
                d["response"], package=package, name=f"{name}Response"
            )

        return ROS2SrvFormat(package, name, definition, req, res)
