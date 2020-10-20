# -*- coding: utf-8 -*-
__all__ = ('NodeConfig', 'ExecutableType',)

import xml.etree.ElementTree as ET
from enum import Enum
from typing import Collection, Optional, Sequence, Tuple

import attr

from ....name import namespace_join


class ExecutableType(Enum):
    PYTHON = 1
    LIKELY_CPP = 2


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeConfig:
    namespace: str
    name: str
    typ: str
    package: str
    executable_path: str
    executable_type: ExecutableType
    remappings: Collection[Tuple[str, str]] = attr.ib(default=())
    filename: Optional[str] = attr.ib(default=None)
    output: Optional[str] = attr.ib(default=None)
    required: bool = attr.ib(default=False)
    respawn: bool = attr.ib(default=False)
    respawn_delay: float = attr.ib(default=0.0)
    env_args: Sequence[Tuple[str, str]] = attr.ib(default=())
    cwd: Optional[str] = attr.ib(default=None)
    args: Optional[str] = attr.ib(default=None)
    launch_prefix: Optional[str] = attr.ib(default=None)

    @property
    def full_name(self) -> str:
        return namespace_join(self.namespace, self.name)

    def with_launch_prefix(self, launch_prefix: str) -> 'NodeConfig':
        return attr.evolve(self, launch_prefix=launch_prefix)

    def with_remappings(self,
                        remappings: Collection[Tuple[str, str]],
                        *,
                        overwrite: bool = False
                        ) -> 'NodeConfig':
        """Applies a set of remappings to this configuration.

        Parameters
        ----------
        remappings: Collection[Tuple[str, str]]
            A collection of name remappings, each given as a tuple of the
            form :code:`(from, to)`.
        overwrite: bool
            If :code:`True`, any existing remappings will not appear in the
            returned variant. Otherwise, the returned variant will contain
            both the supplied and existing remappings.

        Returns
        -------
        NodeConfig
            A variant of this configuration with the given remappings.
        """
        if not overwrite:
            remappings = tuple(self.remappings) + tuple(remappings)
        return attr.evolve(self, remappings=remappings)

    def to_xml_element(self) -> ET.Element:
        element = ET.Element('node')
        if self.package:
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
