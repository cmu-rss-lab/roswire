# -*- coding: utf-8 -*-
__all__ = "ROS1MsgFormat"

from typing import Any, Dict, Optional

import attr
import dockerblade

from ..common import Field, MsgFormat


@attr.s(frozen=True)
class ROS1MsgFormat(MsgFormat[Field]):

    @classmethod
    def _field_from_string(cls, package: str, line: str) -> Optional[Field]:
        return Field.from_string(package, line)

    @classmethod
    def _field_from_dict(cls, dict: Dict[str, Any]) -> Field:
        return Field.from_dict(dict)

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS1MsgFormat":
        mf = super().from_file(package, filename, files)
        assert isinstance(mf, ROS1MsgFormat)
        return mf

    @classmethod
    def from_dict(
            cls,
            d: Dict[str, Any],
            *,
            package: Optional[str] = None,
            name: Optional[str] = None
    ) -> "ROS1MsgFormat":
        mf = super().from_dict(d, package=package, name=name)
        assert isinstance(mf, ROS1MsgFormat)
        return mf
