# -*- coding: utf-8 -*-
__all__ = ("ROS2Field")

import re
from typing import Any, Dict, List, Optional

import attr
import dockerblade

from ..common import Field
from ..common.msg import Constant, MsgFormat, R_BLANK, R_COMMENT
from ..exceptions import ParsingError


@attr.s(frozen=True, str=False, slots=True, auto_attribs=True)
class ROS2Field(Field):
    R_TYPE = r"[a-zA-Z_/][a-zA-Z0-9_/]*(?:<=\d+)?(?:\[(?:<=)?\d*\])?"
    R_DEFAULT_VALUE = r"[^#]*"
    R_FIELD = re.compile(f"^\s*(?P<type>{R_TYPE})"
                         f"\s+(?P<name>{Field.R_NAME})(?:\s+)?"
                         f"(?P<val>{R_DEFAULT_VALUE}){R_COMMENT}")

    default_value: Optional[str]

    @classmethod
    def from_string(cls, package: str, line: str) -> Optional["ROS2Field"]:
        m_field = cls.R_FIELD.match(line)
        if m_field:
            typ = m_field.group('type')
            name = m_field.group('name')
            typ = cls._resolve_type(package, typ)
            default_value = m_field.group('val')
            field = ROS2Field(typ,
                              name,
                              default_value if default_value else None)
            return field
        return None


class ROS2MsgFormat(MsgFormat[ROS2Field, Constant]):

    @classmethod
    def from_string(
        cls, package: str, name: str, text: str
    ) -> "ROS2MsgFormat":
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
                raise ParsingError(f"failed to parse line: {line}\nfrom message:\n{text}")

        return ROS2MsgFormat(package=package,
                             name=name,
                             definition=text,
                             fields=fields,
                             constants=constants)

    @classmethod
    def _field_from_string(cls, package: str, line: str) -> Optional[Field]:
        return ROS2Field.from_string(package, line)

    @classmethod
    def _field_from_dict(cls, dict: Dict[str, Any]) -> Field:
        return ROS2Field.from_dict(dict)

    @classmethod
    def from_file(
        cls, package: str, filename: str, files: dockerblade.FileSystem
    ) -> "ROS2MsgFormat":
        mf = super().from_file(package, filename, files)
        assert isinstance(mf, ROS2MsgFormat)
        return mf

    @classmethod
    def from_dict(
        cls,
        d: Dict[str, Any],
        *,
        package: Optional[str] = None,
        name: Optional[str] = None
    ) -> "ROS2MsgFormat":
        mf = super().from_dict(d, package=package, name=name)
        assert isinstance(mf, ROS2MsgFormat)
        return mf
