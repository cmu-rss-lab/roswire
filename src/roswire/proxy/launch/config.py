# -*- coding: utf-8 -*-
"""
This file provides data structures that represent ROS launch configurations.
"""
__all__ = ('ROSConfig', 'NodeConfig')

from typing import Tuple, FrozenSet, Optional, Dict, Any
import logging

import attr

from ...exceptions import FailedToParseLaunchFile
from ...name import (namespace_join, canonical_name, name_is_global,
                     namespaces_of)

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(frozen=True, slots=True)
class Parameter:
    name: str = attr.ib()
    value: str = attr.ib()  # TODO convert to appropriate type


@attr.s(frozen=True, slots=True)
class NodeConfig:
    namespace: str = attr.ib()
    name: str = attr.ib()
    typ: str = attr.ib()
    package: str = attr.ib()
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

    def with_param(self, name: str, value: Any) -> 'ROSConfig':
        """Adds a parameter to this configuration."""
        params = self.params.copy()
        errors = self.errors

        if not name_is_global(name):
            m = f"expected parameter name to be global: {name}"
            raise FailedToParseLaunchFile(m)

        for parent_name in (n for n in namespaces_of(name) if n in params):
            err = f"parameter [{name}] conflicts with parent [{parent_name}]"
            errors = errors + (err,)

        params[name] = value
        return attr.evolve(self, params=params, errors=errors)

    def with_executable(self, executable: str) -> 'ROSConfig':
        """Specify an executable that should be run at launch."""
        executables = self.executables + (executable,)
        return attr.evolve(self, executables=executables)

    def with_roslaunch_file(self, filename: str) -> 'ROSConfig':
        roslaunch_files = self.roslaunch_files + (filename,)
        return attr.evolve(self, roslaunch_files=roslaunch_files)

    def with_node(self, node: NodeConfig) -> 'ROSConfig':
        logger.debug("adding node to config: %s", node)
        full_name = node.full_name
        used_names = {n.full_name for n in self.nodes}
        if node.full_name in used_names:
            m = 'multiple definitions of node [{}] in launch configuration'
            m = m.format(node.full_name)
            raise FailedToParseLaunchFile(m)
        nodes = self.nodes | frozenset({node})
        return attr.evolve(self, nodes=nodes)
