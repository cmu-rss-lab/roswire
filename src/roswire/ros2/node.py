# -*- coding: utf-8 -*-
__all__ = ('ROS2Node',)

import typing
import attr

from ..interface import Node
from .state import ROS2StateProbe

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
    name: str = attr.ib()
    _state_probe: 'ROS2StateProbe' = attr.ib(init=False)

    @classmethod
    def for_app_instance_and_name(cls,
                                  app_instance: 'AppInstance',
                                  name: str
                                  ) -> 'ROS2Node':
        return ROS2Node(name=name, app_instance=app_instance)

    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        raise NotImplementedError

    def shutdown(self, ignore_errors: bool = False) -> None:
        """Instructs this node to shutdown."""
        raise NotImplementedError
