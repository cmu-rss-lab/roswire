__all__ = ['Constant', 'ConstantValue', 'Field', 'MsgFormat']

from typing import Type, Optional, Any, Union, Tuple
import re

import attr

R_TYPE = r"[a-zA-Z0-9_/]+"
R_NAME = r"[a-zA-Z0-9_/]+"
R_VAL = r".+"
R_COMMENT = r"(#.*)?"
R_FIELD = re.compile(
    f"^({R_TYPE})\s+({R_NAME})\s*{R_COMMENT}$")
# R_CONSTANT = re.compile(
#     f"^({R_TYPE})\s+({R_NAME})=\s*({R_VAL})\s*$")
R_CONSTANT = re.compile(f"^(\w+)\s+(\w+)=(.+)$")
R_BLANK = re.compile(
    f"^\s*{R_COMMENT}$")

ConstantValue = Union[str, int, float]


def is_legal_constant_type(name_typ: str) -> bool:
    """
    Determines whether a type, given by its name, may be used to provide a
    constant.
    """
    raise NotImplementedError


def is_legal_msg_type(name_typ: str) -> bool:
    raise NotImplementedError


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
    def from_string(text: str, package: str, name: str) -> 'MsgFormat':
        """
        Constructs a message format from the contents of a .msg file.
        Reference:
            https://github.com/strawlab/ros/blob/master/core/roslib/src/roslib/msgs.py#L577
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
                typ, name, val_str = m_constant.group(1, 2, 3)

                # is the type valid?
                # if not is_legal_constant_type(typ):
                #     raise Exception(f"illegal constant type: {typ}")

                # FIXME convert value
                val: ConstantValue = val_str

                constant: Constant = Constant(typ, name, val)
                constants.append(constant)
            elif m_field:
                typ, name = m_field.group(1, 2)
                # if not is_legal_type(typ):
                #     raise Exception(f"illegal type: {type}")
                field: Field = Field(typ, name)
                fields.append(field)
            else:
                raise Exception(f"failed to parse line: {line}")

        return MsgFormat(package, name, fields, constants)
