# -*- coding: utf-8 -*-
__all__ = ('ROS2Node',)

import typing

import attr

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2Node():
    """Provides an interface for interacting with a ROS2 node.

    Attributes
    ----------
    name: str
        The name of the node.
    """
    app_instance: 'AppInstance'

    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        raise NotImplementedError

    def shutdown(self) -> None:
        """Instructs this node to shutdown."""
        raise NotImplementedError
