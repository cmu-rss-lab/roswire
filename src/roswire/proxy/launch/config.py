# -*- coding: utf-8 -*-
"""
This file provides data structures that represent ROS launch configurations.
"""
__all__ = ('ROSConfig', 'NodeConfig')

from typing import Tuple

import attr


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


@attr.s(frozen=True, slots=True)
class ROSConfig:
    nodes: Tuple[str, ...] = attr.ib(default=tuple())
    executables: Tuple[str, ...] = attr.ib(default=tuple())
    roslaunch_files: Tuple[str, ...] = attr.ib(default=tuple())

    def with_executable(self, executable: str) -> 'ROSConfig':
        """Specify an executable that should be run at launch."""
        executables = self.executables + (executable,)
        return attr.evolve(self, executables=executables)

    def with_roslaunch_file(self, filename: str) -> 'ROSConfig':
        roslaunch_files = self.roslaunch_files + (filename,)
        return attr.evolve(self, roslaunch_files=roslaunch_files)

    def with_node(self, name: str) -> 'ROSConfig':
        nodes = self.nodes + (name,)
        return attr.evolve(self, nodes=nodes)
