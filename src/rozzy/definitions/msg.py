__all__ = ['Constant', 'ConstantValue', 'Field', 'MsgFormat']

from typing import Type, Optional, Any, Union, Tuple
import re

import attr

from .. import exceptions

R_TYPE = r"[a-zA-Z0-9_/]+"
R_NAME = r"[a-zA-Z0-9_/]+"
R_VAL = r".+"
R_COMMENT = r"(#.*)?"
R_FIELD = re.compile(f"^\s*({R_TYPE})\s+({R_NAME})\s*{R_COMMENT}$")
R_CONSTANT = re.compile(f"^\s*(\w+)\s+(\w+)=(.+)$")
R_BLANK = re.compile(
    f"^\s*{R_COMMENT}$")

ConstantValue = Union[str, int, float]


@attr.s(frozen=True)
class Constant:
    typ = attr.ib(type=str)
    name = attr.ib(type=str)
    value = attr.ib(type=ConstantValue)


@attr.s(frozen=True)
class Field:
    typ = attr.ib(type=str)
    name = attr.ib(type=str)


@attr.s(frozen=True)
class MsgFormat:
    package = attr.ib(type=str)
    name = attr.ib(type=str)
    fields = attr.ib(type=Tuple[Field, ...], converter=tuple)
    constants = attr.ib(type=Tuple[Constant, ...], converter=tuple)

    @staticmethod
    def from_string(package: str, name: str, text: str) -> 'MsgFormat':
        """
        Constructs a message format from its description.

        Raises:
            ParsingError: if the description cannot be parsed.
        """
        fields: List[Field] = []
        constants: List[Constant] = []

        for line in text.split('\n'):
            m_blank: re.Match = R_BLANK.match(line)
            m_constant: re.Match = R_CONSTANT.match(line)
            m_field: re.Match = R_FIELD.match(line)

            if m_blank:
                continue
            elif m_constant:
                typ, name_const, val_str = m_constant.group(1, 2, 3)

                # FIXME convert value
                val: ConstantValue = val_str

                constant: Constant = Constant(typ, name_const, val)
                constants.append(constant)
            elif m_field:
                typ, name_field = m_field.group(1, 2)
                field: Field = Field(typ, name_field)
                fields.append(field)
            else:
                raise exceptions.ParsingError(f"failed to parse line: {line}")

        return MsgFormat(package, name, fields, constants)
