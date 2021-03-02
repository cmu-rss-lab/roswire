# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
__all__ = ("ROS2MsgFormat", "ROS2Message")

from typing import Any, Dict, List, Optional

import re

import attr

from ..common import Constant, Field, Message, MsgFormat
from ..common.msg import R_BLANK
from ..exceptions import ParsingError


@attr.s(frozen=True, str=False, slots=True, auto_attribs=True)
class ROS2Field(Field):
    R_DEFAULT_VALUE = r"[^#]*"
    R_FIELD = re.compile(f"^\s*(?P<type>{Field.R_TYPE})"
                         f"\s+(?P<name>{Field.R_NAME})(?:\s+)?"
                         f"(?P<val>{R_DEFAULT_VALUE}){Field.R_COMMENT}")

    val: Optional[str]

    @classmethod
    def from_string(cls, package: str, line: str) -> Optional["ROS2Field"]:
        m_field = cls.R_FIELD.match(line)
        if m_field:
            typ = m_field.group('type')
            name = m_field.group('name')
            typ = cls._resolve_type(package, typ)
            val = m_field.group('val')
            field = ROS2Field(type, name, val if val and val != '' else None)
            return field
        return None


class ROS2MsgFormat(MsgFormat[ROS2Field, Constant]):

    @classmethod
    def from_string(cls, package: str, name: str, text: str) -> "MsgFormat":
        typ: str
        name_const: str
        fields: List[ROS2Field] = []
        constants: List[Constant] = []

        for line in text.split("\n"):
            m_blank = R_BLANK.match(line)
            if m_blank:
                continue

            constant = Constant.from_string(line)
            field = ROS2Field.from_string(package, line)
            if constant:
                constants.append(constant)
            elif field:
                fields.append(field)
            else:
                raise ParsingError(f"failed to parse line: {line}")

        return ROS2MsgFormat(package, name, text, fields, constants)  # type: ignore  # noqa

    @classmethod
    def from_dict(
        cls,
        d: Dict[str, Any],
        *,
        package: Optional[str] = None,
        name: Optional[str] = None,
    ) -> "ROS2MsgFormat":
        if not package:
            package = d["package"]
        if not name:
            name = d["name"]
        definition = d["definition"]
        fields = [ROS2Field.from_dict(dd) for dd in d.get("fields", [])]
        constants = [Constant.from_dict(dd) for dd in d.get("constants", [])]
        return ROS2MsgFormat(package, name, definition, fields, constants)  # type: ignore  # noqa


class ROS2Message(Message[ROS2MsgFormat]):
    ...
