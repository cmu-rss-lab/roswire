# -*- coding: utf-8 -*-
__all__ = ("Env",)

import xml.etree.ElementTree as ET

import attr


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Env:
    """Provides an environment variable definition.

    Attributes
    ----------
    name: str
        The name of the environment variable.
    value: str
        The concrete value of the environment variable.
    """

    name: str
    value: str

    def to_xml_element(self) -> ET.Element:
        element = ET.Element("env")
        element.attrib["name"] = self.name
        element.attrib["value"] = self.value
        return element
