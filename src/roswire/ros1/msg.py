# -*- coding: utf-8 -*-
__all__ = ("ROS1MsgFormat", "ROS1Message")

from typing import Any, Dict, List, Optional

from ..common import Constant, Field, Message, MsgFormat
from ..common.msg import R_BLANK
from ..exceptions import ParsingError


class ROS1MsgFormat(MsgFormat[Field, Constant]):

    @classmethod
    def from_string(cls, package: str, name: str, text: str) -> "MsgFormat":
        typ: str
        name_const: str
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

        return ROS1MsgFormat(package, name, text, fields, constants)  # type: ignore  # noqa

    @classmethod
    def from_dict(
        cls,
        d: Dict[str, Any],
        *,
        package: Optional[str] = None,
        name: Optional[str] = None,
    ) -> "MsgFormat":
        if not package:
            package = d["package"]
        if not name:
            name = d["name"]
        definition = d["definition"]
        fields = [Field.from_dict(dd) for dd in d.get("fields", [])]
        constants = [Constant.from_dict(dd) for dd in d.get("constants", [])]
        return MsgFormat(package, name, definition, fields, constants)  # type: ignore  # noqa


class ROS1Message(Message[ROS1MsgFormat]):
    ...
