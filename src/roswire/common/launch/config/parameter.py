# -*- coding: utf-8 -*-
__all__ = ("Parameter",)

import xml.etree.ElementTree as ET
from typing import Any, Optional

import attr
import yaml


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Parameter:
    """Provides a ROS parameter definition.

    Attributes
    ----------
    name: str
        The name of the parameter.
    typ: str
        The name of the type used by the parameter.
    command: str, optional
        The command that was used to define the parameter, if known.
    value: Any
        The value of the parameter.
    """

    name: str
    typ: str
    value: Any
    command: Optional[str] = attr.ib(default=None)

    def to_xml_element(self) -> ET.Element:
        # primitive parameters
        if type(self.value) in (str, float, int, bool):
            element = ET.Element("param")
            element.attrib["name"] = self.name

            if self.typ != "auto":
                element.attrib["type"] = self.typ

            if self.command:
                element.attrib["command"] = self.command
            elif isinstance(self.value, bool):
                element.attrib["value"] = "true" if self.value else "false"
            else:
                element.attrib["value"] = str(self.value)
            return element

        # complex parameters
        element = ET.Element("rosparam")
        element.text = yaml.safe_dump(self.value, default_flow_style=True)
        element.attrib["param"] = self.name
        return element
