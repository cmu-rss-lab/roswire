# -*- coding: utf-8 -*-
__all__ = ("ROS2Field")

import re
from typing import Any, Dict, Optional

import attr
import dockerblade

from ..common import Field
from ..common.msg import MsgFormat, R_COMMENT


@attr.s(frozen=True, str=False, slots=True, auto_attribs=True)
class ROS2Field(Field):
    R_DEFAULT_VALUE = r"[^#]*"
    R_FIELD = re.compile(f"^\s*({Field.R_TYPE})(\s+)({Field.R_NAME})"
                         f"\s+({R_DEFAULT_VALUE})\s*"
                         f"{R_COMMENT}$")

    default_val: Optional[str]

    @classmethod
    def from_string(cls, packages: str, line: str) -> Optional["ROS2Field"]:
        old_field = Field.from_string(packages, line)
        m_field = cls.R_FIELD.match(line)
        if old_field and m_field:
            def_val = m_field.group(4)

            field: ROS2Field = ROS2Field(old_field.typ,
                                         old_field.name,
                                         def_val)
            return field
        return None

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ROS2Field":
        value: Optional[str] = None
        if 'default' in d:
            value = d['default']
        return ROS2Field(d['type'], d['name'], value)

    def to_dict(self) -> Dict[str, Any]:
        d = {'type': self.typ, 'name': self.name}
        if self.default_val:
            d['default'] = self.default_val
        return d

    def __str__(self) -> str:
        s = super().__str__()
        if self.default_val:
            s = f"{s} {self.default_val}"
        return s


@attr.s(frozen=True)
class ROS2MsgFormat(MsgFormat[ROS2Field]):

    @classmethod
    def _field_from_dict(cls, dict: Dict[str, Any]) -> ROS2Field:
        return ROS2Field.from_dict(dict)

    @classmethod
    def _field_from_string(
        cls, package: str, line: str
    ) -> Optional[ROS2Field]:
        return ROS2Field.from_string(package, line)

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
