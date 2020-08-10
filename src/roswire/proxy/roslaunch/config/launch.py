# -*- coding: utf-8 -*-
__all__ = ('LaunchConfig',)

from typing import (AbstractSet, Any, Collection, Dict, Mapping, Optional,
                    Set, Sequence, Tuple)
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET

from loguru import logger
import attr

from .node import NodeConfig
from .env import Env
from .parameter import Parameter
from ....exceptions import FailedToParseLaunchFile
from ....name import (canonical_name, name_is_global, namespace_join,
                      namespaces_of)


@attr.s(frozen=True, slots=True)
class LaunchConfig:
    nodes: AbstractSet[NodeConfig] = attr.ib(default=frozenset(),
                                             converter=frozenset)
    executables: Sequence[str] = attr.ib(default=tuple())
    roslaunch_files: Sequence[str] = attr.ib(default=tuple())
    params: Mapping[str, Any] = attr.ib(factory=dict)
    clear_params: Sequence[str] = attr.ib(default=tuple())
    errors: Sequence[str] = attr.ib(default=tuple())
    envs: Mapping[str, Env] = attr.ib(default=dict())

    def with_clear_param(self, ns: str) -> 'LaunchConfig':
        """Specifies a parameter that should be cleared before new parameters
        are set."""
        ns = canonical_name(ns)
        if ns in self.clear_params:
            return self
        clear_params = tuple(self.clear_params) + (ns,)
        return attr.evolve(self, clear_params=clear_params)

    def with_launch_prefixes(self,
                             prefixes: Mapping[str, str]
                             ) -> 'LaunchConfig':
        nodes: Dict[str, NodeConfig] = {n.name: n for n in self.nodes}
        for node_name, prefix in prefixes.items():
            nodes[node_name] = nodes[node_name].with_launch_prefix(prefix)
        return attr.evolve(self, nodes=frozenset(nodes.values()))

    def with_env(self,
                 name: str,
                 value: str
                 ) -> 'LaunchConfig':
        """Adds an environment variable to this configuration."""
        envs: Dict[str, Env] = dict(self.envs)
        envs[name] = Env(name=name, value=value)
        return attr.evolve(self, envs=envs)

    def with_rosparam(self,
                      name: str,
                      value: Any
                      ) -> 'LaunchConfig':
        # complex parameters
        if isinstance(value, dict):
            config: LaunchConfig = self
            for child_name, child_value in value.items():
                child_name = namespace_join(name, child_name)
                config = config.with_rosparam(child_name, child_value)
            return config

        # primitive parameters
        # FIXME better handling of types
        return self.with_param(name, typ='auto', value=value)

    def with_param(self,
                   name: str,
                   typ: str,
                   value: Any,
                   command: Optional[str] = None
                   ) -> 'LaunchConfig':
        """Adds a parameter to this configuration."""
        param = Parameter(name=name, typ=typ, value=value, command=command)
        params: Dict[str, Any] = dict(self.params)
        errors = self.errors

        if not name_is_global(name):
            m = f"expected parameter name to be global: {name}"
            raise FailedToParseLaunchFile(m)

        for parent_name in (n for n in namespaces_of(name) if n in params):
            err = f"parameter [{name}] conflicts with parent [{parent_name}]"
            errors = tuple(errors) + (err,)

        params[name] = param
        return attr.evolve(self, params=params, errors=errors)

    def with_remappings(self,
                        node_to_remappings: Mapping[str, Collection[Tuple[str, str]]]  # noqa
                        ) -> 'LaunchConfig':
        nodes: Set[NodeConfig] = set()
        for node in self.nodes:
            if node.name in node_to_remappings:
                node = node.with_remappings(node_to_remappings[node.name])
            nodes.add(node)
        return attr.evolve(self, nodes=frozenset(nodes))

    def with_executable(self, executable: str) -> 'LaunchConfig':
        """Specify an executable that should be run at launch."""
        executables = tuple(self.executables) + (executable,)
        return attr.evolve(self, executables=executables)

    def with_roslaunch_file(self, filename: str) -> 'LaunchConfig':
        roslaunch_files = tuple(self.roslaunch_files) + (filename,)
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
        for env in self.envs.values():
            root.append(env.to_xml_element())
        for param in self.params.values():
            root.append(param.to_xml_element())
        for node in self.nodes:
            root.append(node.to_xml_element())
        return ET.ElementTree(root)

    def to_xml_string(self) -> str:
        """Transforms this launch configuration to a launch XML file."""
        xml = self.to_xml_tree()
        contents = ET.tostring(xml.getroot())
        return minidom.parseString(contents).toprettyxml(indent='  ')

    def to_xml_file(self, filename: str) -> None:
        """Saves this launch configuration to an equivalent launch XML file."""
        contents = self.to_xml_string()
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(contents)
