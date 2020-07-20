# -*- coding: utf-8 -*-
__all__ = ('ROS2Node',)

import typing

import attr

from ..interface import Node

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
    __name: str

    @classmethod
    def for_app_instance_and_name(cls,
                                  name: str,
                                  app_instance: 'AppInstance'
                                  ) -> 'ROS2Node':
        return ROS2Node(name=name, app_instance=app_instance)

    @property
    def name(self) -> str:
        return self.__name

    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        raise NotImplementedError

    def shutdown(self) -> None:
        """Instructs this node to shutdown."""
        raise NotImplementedError
