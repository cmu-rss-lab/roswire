# -*- coding: utf-8 -*-
"""
This file provides data structures that represent ROS launch configurations.
"""
__all__ = ('ROSConfig', 'NodeConfig')

from typing import Tuple, FrozenSet
import logging

import attr

from ...exceptions import FailedToParseLaunchFile
from ...name import namespace_join

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
    pkg: str = attr.ib()
    required: bool = attr.ib(default=False)

    @property
    def full_name(self) -> str:
        return namespace_join(self.namespace, self.name)


@attr.s(frozen=True, slots=True)
class ROSConfig:
    nodes: FrozenSet[NodeConfig] = attr.ib(default=frozenset(),
                                           converter=frozenset)
    executables: Tuple[str, ...] = attr.ib(default=tuple())
    roslaunch_files: Tuple[str, ...] = attr.ib(default=tuple())

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
