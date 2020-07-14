# -*- coding: utf-8 -*-
__all__ = ('ROS2NodeManager',)

from typing import Iterator, Mapping
import typing

from loguru import logger
import attr

from .node import ROS2Node
from .. import exceptions as exc

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2NodeManager(Mapping[str, ROS2Node]):
    """Provides an interface for interacting with ROS2 nodes."""
    app_instance: 'AppInstance' = attr.ib()

    @classmethod
    def for_app_instance(cls,
                         app_instance: 'AppInstance'
                         ) -> 'ROS2NodeManager':
        return ROS2NodeManager(app_instance=app_instance)

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
        raise NotImplementedError

    def __len__(self) -> int:
        """Returns a count of the number of active nodes."""
        raise NotImplementedError

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all active nodes."""
        raise NotImplementedError

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
