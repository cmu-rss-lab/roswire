# -*- coding: utf-8 -*-
__all__ = ('LaunchConfig',)

from typing import Tuple, FrozenSet, Optional, Dict, Any
import xml.etree.ElementTree as ET

from loguru import logger
import attr

from .node import NodeConfig
from .parameter import Parameter
from ....exceptions import FailedToParseLaunchFile
from ....name import (namespace_join, canonical_name, name_is_global,
                      namespaces_of)


@attr.s(frozen=True, slots=True)
class LaunchConfig:
    nodes: FrozenSet[NodeConfig] = attr.ib(default=frozenset(),
                                           converter=frozenset)
    executables: Tuple[str, ...] = attr.ib(default=tuple())
    roslaunch_files: Tuple[str, ...] = attr.ib(default=tuple())
    params: Dict[str, Any] = attr.ib(factory=dict)
    clear_params: Tuple[str, ...] = attr.ib(default=tuple())
    errors: Tuple[str, ...] = attr.ib(default=tuple())

    def with_clear_param(self, ns: str) -> 'LaunchConfig':
        """
        Specifies a parameter that should be cleared before new parameters
        are set.
        """
        ns = canonical_name(ns)
        if ns in self.clear_params:
            return self
        clear_params = self.clear_params + (ns,)
        return attr.evolve(self, clear_params=clear_params)

    def with_param(self, name: str, typ: str, value: Any) -> 'LaunchConfig':
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

    def with_executable(self, executable: str) -> 'LaunchConfig':
        """Specify an executable that should be run at launch."""
        executables = self.executables + (executable,)
        return attr.evolve(self, executables=executables)

    def with_roslaunch_file(self, filename: str) -> 'LaunchConfig':
        roslaunch_files = self.roslaunch_files + (filename,)
        return attr.evolve(self, roslaunch_files=roslaunch_files)

    def with_node(self, node: NodeConfig) -> 'LaunchConfig':
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
