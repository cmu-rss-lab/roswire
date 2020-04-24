# -*- coding: utf-8 -*-
"""
This file provides data structures that represent ROS launch configurations.
"""
__all__ = ('ROSConfig', 'NodeConfig')

from typing import Tuple, FrozenSet, Optional, Dict, Any
import xml.etree.ElementTree as ET

from loguru import logger
import attr
import yaml

try:
    from yaml import Dumper as YamlDumper  # type: ignore
except ImportError:
    from yaml import CDumper as YamlDumper   # type: ignore

from ...exceptions import FailedToParseLaunchFile
from ...name import (namespace_join, canonical_name, name_is_global,
                     namespaces_of)


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
        print(self)
        element.attrib['value'] = value
        return element


@attr.s(frozen=True, slots=True, auto_attribs=True)
class NodeConfig:
    namespace: str
    name: str
    typ: str
    package: str
    remappings: Tuple[Tuple[str, str], ...] = attr.ib(default=tuple())
    filename: Optional[str] = attr.ib(default=None)
    output: Optional[str] = attr.ib(default=None)
    required: bool = attr.ib(default=False)
    respawn: bool = attr.ib(default=False)
    respawn_delay: float = attr.ib(default=0.0)
    env_args: Tuple[Tuple[str, str], ...] = attr.ib(default=tuple())
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


@attr.s(frozen=True, slots=True)
class ROSConfig:
    nodes: FrozenSet[NodeConfig] = attr.ib(default=frozenset(),
                                           converter=frozenset)
    executables: Tuple[str, ...] = attr.ib(default=tuple())
    roslaunch_files: Tuple[str, ...] = attr.ib(default=tuple())
    params: Dict[str, Any] = attr.ib(factory=dict)
    clear_params: Tuple[str, ...] = attr.ib(default=tuple())
    errors: Tuple[str, ...] = attr.ib(default=tuple())

    def with_clear_param(self, ns: str) -> 'ROSConfig':
        """
        Specifies a parameter that should be cleared before new parameters
        are set.
        """
        ns = canonical_name(ns)
        if ns in self.clear_params:
            return self
        clear_params = self.clear_params + (ns,)
        return attr.evolve(self, clear_params=clear_params)

    def with_param(self, name: str, typ: str, value: Any) -> 'ROSConfig':
        """Adds a parameter to this configuration."""
        param = Parameter(name=name, typ=typ, value=value)
        params = self.params.copy()
        errors = self.errors

        if not name_is_global(name):
            m = f"expected parameter name to be global: {name}"
            raise FailedToParseLaunchFile(m)

        for parent_name in (n for n in namespaces_of(name) if n in params):
            err = f"parameter [{name}] conflicts with parent [{parent_name}]"
            errors = errors + (err,)

        params[name] = param
        return attr.evolve(self, params=params, errors=errors)

    def with_executable(self, executable: str) -> 'ROSConfig':
        """Specify an executable that should be run at launch."""
        executables = self.executables + (executable,)
        return attr.evolve(self, executables=executables)

    def with_roslaunch_file(self, filename: str) -> 'ROSConfig':
        roslaunch_files = self.roslaunch_files + (filename,)
        return attr.evolve(self, roslaunch_files=roslaunch_files)

    def with_node(self, node: NodeConfig) -> 'ROSConfig':
        logger.debug(f"adding node to config: {node}")
        used_names = {n.full_name for n in self.nodes}
        if node.full_name in used_names:
            m = 'multiple definitions of node [{}] in launch configuration'
            m = m.format(node.full_name)
            raise FailedToParseLaunchFile(m)
        nodes = self.nodes | frozenset({node})
        return attr.evolve(self, nodes=nodes)

    def to_xml_tree(self) -> ET.ElementTree:
        root = ET.Element('launch')
        for param in self.params.values():
            root.append(param.to_xml_element())
        for node in self.nodes:
            root.append(node.to_xml_element())
        return ET.ElementTree(root)
