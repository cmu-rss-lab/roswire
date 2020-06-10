# -*- coding: utf-8 -*-
__all__ = ('NodeConfig',)

from typing import Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import attr

from ....name import namespace_join


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeConfig:
    namespace: str
    name: str
    typ: str
    package: str
    remappings: Sequence[Tuple[str, str]] = attr.ib(default=tuple())
    filename: Optional[str] = attr.ib(default=None)
    output: Optional[str] = attr.ib(default=None)
    required: bool = attr.ib(default=False)
    respawn: bool = attr.ib(default=False)
    respawn_delay: float = attr.ib(default=0.0)
    env_args: Sequence[Tuple[str, str]] = attr.ib(default=tuple())
    cwd: Optional[str] = attr.ib(default=None)
    args: Optional[str] = attr.ib(default=None)
    launch_prefix: Optional[str] = attr.ib(default=None)

    @property
    def full_name(self) -> str:
        return namespace_join(self.namespace, self.name)

    def to_xml_element(self) -> ET.Element:
        element = ET.Element('node')
        element.attrib['pkg'] = self.package
        element.attrib['type'] = self.typ
        element.attrib['name'] = self.name
        element.attrib['ns'] = self.namespace
        element.attrib['respawn'] = str(self.respawn)
        element.attrib['respawn_delay'] = str(self.respawn_delay)
        element.attrib['required'] = str(self.required)
        if self.launch_prefix:
            element.attrib['launch-prefix'] = self.launch_prefix
        if self.args:
            element.attrib['args'] = self.args
        if self.cwd:
            element.attrib['cwd'] = self.cwd
        if self.output:
            element.attrib['output'] = self.output
        for remap_from, remap_to in self.remappings:
            attrib = {'from': remap_from, 'to': remap_to}
            ET.SubElement(element, 'remap', attrib=attrib)
        return element
