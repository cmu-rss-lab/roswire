# -*- coding: utf-8 -*-
__all__ = ('ROS2NodeManager',)

from typing import Iterator
import typing

from loguru import logger
import attr

from .node import ROS2Node
from .state import ROS2StateProbe
from ..proxy import SystemState
from .. import exceptions as exc
from ..interface import NodeManager

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2NodeManager(NodeManager):
    """Provides an interface for interacting with ROS2 nodes."""
    app_instance: 'AppInstance' = attr.ib()
    _state_probe: 'ROS2StateProbe' = attr.ib(init=False)

    @classmethod
    def for_app_instance(cls,
                         app_instance: 'AppInstance'
                         ) -> 'ROS2NodeManager':
        return ROS2NodeManager(app_instance=app_instance)

    @property
    def state(self) -> 'SystemState':
        return self._state_probe.probe()

    def __getitem__(self, name: str) -> ROS2Node:
        """Attempts to fetch a given node.

        Parameters
        ----------
        name: str
            The name of the node.

        Returns
        -------
        ROS2Node
            An interface to the given node.

        Raises
        ------
        NodeNotFoundError
            If there is no node with the given name.
        """
        return ROS2Node.for_app_instance_and_name(self.app_instance, name)

    def __len__(self) -> int:
        """Returns a count of the number of active nodes."""
        return len(self.state.nodes)

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all active nodes."""
        yield from self.state.nodes

    def __delitem__(self, name: str) -> None:
        """Shutdown and deregister a given node.

        Parameters
        ----------
        name: str
            The name of the node.

        Raises
        ------
        NodeNotFoundError
            If there is no node with the given name.
        """
        try:
            node = self[name]
        except exc.NodeNotFoundError:
            logger.exception(f"failed to delete node [{name}]: not found.")
            raise
        node.shutdown()
