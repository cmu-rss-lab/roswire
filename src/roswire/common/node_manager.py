# -*- coding: utf-8 -*-
__all__ = ("NodeManager",)

import abc
from typing import Iterator, Mapping

from .node import Node


class NodeManager(Mapping[str, Node], abc.ABC):
    """Provides a common interface for interacting with ROS1 and RO2 nodes."""

    @abc.abstractmethod
    def __getitem__(self, name: str) -> Node:
        """Attempts to fetch a given node.

        Parameters
        ----------
        name: str
            The name of the node.

        Returns
        -------
        Node
            An interface to the given node.

        Raises
        ------
        NodeNotFoundError
            If there is no node with the given name.
        """
        ...

    @abc.abstractmethod
    def __len__(self) -> int:
        """Returns a count of the number of active nodes."""
        ...

    @abc.abstractmethod
    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all active nodes."""
        ...

    @abc.abstractmethod
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
        ...
