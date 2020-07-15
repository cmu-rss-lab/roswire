# -*- coding: utf-8 -*-
__all__ = ('Node',)

import abc


class Node(abc.ABC):
    """Provides a common interface for a particular ROS1 or RO2 node.

    Attributes
    ----------
    name: str
        The fully qualified name of this node.
    """
    @property
    @abc.abstractmethod
    def name(self) -> str:
        ...

    @abc.abstractmethod
    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        ...

    def shutdown(self) -> None:
        """Instructs this node to shutdown."""
        ...
