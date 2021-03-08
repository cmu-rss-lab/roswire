# -*- coding: utf-8 -*-
__all__ = "ROS1MsgFormat"

from typing import Any, Dict, List, Optional

import dockerblade

from ..common import Constant, Field, MsgFormat
from ..common.msg import R_BLANK
from ..exceptions import ParsingError


class ROS1MsgFormat(MsgFormat[Field, Constant]):

    @classmethod
    def from_string(
        cls, package: str, name: str, text: str
    ) -> "ROS1MsgFormat":
        fields: List[Field] = []
        constants: List[Constant] = []

        for line in text.split("\n"):
            m_blank = R_BLANK.match(line)
            if m_blank:
                continue

            constant = Constant.from_string(line)
            field = Field.from_string(package, line)
            if constant:
                constants.append(constant)
            elif field:
                fields.append(field)
            else:
                raise ParsingError(f"failed to parse line: {line}")
        return cls(package, name, text, fields, constants)  # type: ignore  # noqa

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
