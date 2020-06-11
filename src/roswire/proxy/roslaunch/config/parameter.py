# -*- coding: utf-8 -*-
__all__ = ('Parameter',)

from typing import Any, Optional
import xml.etree.ElementTree as ET

import attr
import yaml

try:
    from yaml import Dumper as YamlDumper  # type: ignore
except ImportError:
    from yaml import CDumper as YamlDumper   # type: ignore


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
        element = ET.Element('param')
        element.attrib['name'] = self.name
        element.attrib['type'] = self.typ
        if self.command:
            element.attrib['command'] = self.command
        elif self.typ in ('int', 'double', 'bool', 'auto'):
            element.attrib['value'] = str(self.value)
        elif self.typ == 'yaml':
            element.attrib['value'] = yaml.dump(self.value, Dumper=YamlDumper)
        else:
            element.attrib['value'] = self.value
        return element
