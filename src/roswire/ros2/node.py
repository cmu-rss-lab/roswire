# -*- coding: utf-8 -*-
__all__ = ('ROS2Node',)

import typing

import attr
import logger

from ..interface import Node
from .state import ROS2StateProbe
import dockerblade

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2Node(Node):
    """Provides an interface for interacting with a ROS2 node.

    Attributes
    ----------
    name: str
        The name of the node.
    """
    app_instance: 'AppInstance' = attr.ib()
    node_name: str = attr.ib()
    _state_probe: 'ROS2StateProbe' = attr.ib(init=False)

    @classmethod
    def for_app_instance_and_name(cls,
                                  app_instance: 'AppInstance',
                                  name: str
                                  ) -> 'ROS2Node':
        return ROS2Node(node_name=name, app_instance=app_instance)

    @property
    def name(self) -> str:
        return self.node_name

    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        if self.node_name in self._state_probe.probe().nodes:
            return True
        return False

    def shutdown(self) -> None:
        """Instructs this node to shutdown."""
        command = f"ros2 lifecycle set {self.node_name} shutdown"
        try:
            self.app_instance.shell.run(command)
        except dockerblade.exceptions.CalledProcessError:
            logger.debug(f"Unable to shutdown node {self.node_name}")
