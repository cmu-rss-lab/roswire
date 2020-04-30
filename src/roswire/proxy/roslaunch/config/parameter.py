# -*- coding: utf-8 -*-
__all__ = ('Parameter',)

from typing import Any
import xml.etree.ElementTree as ET

import attr
import yaml

try:
    from yaml import Dumper as YamlDumper  # type: ignore
except ImportError:
    from yaml import CDumper as YamlDumper   # type: ignore


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Parameter:
    name: str
    typ: str
    value: Any

    def to_xml_element(self) -> ET.Element:
        element = ET.Element('param')
        element.attrib['name'] = self.name
        element.attrib['type'] = self.typ
        if self.typ in ('int', 'double', 'bool', 'auto'):
            value = str(self.value)
        elif self.typ == 'yaml':
            value = yaml.dump(self.value, Dumper=YamlDumper)
        else:
            value = self.value
        element.attrib['value'] = value
        return element
