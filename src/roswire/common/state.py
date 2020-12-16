# -*- coding: utf-8 -*-
__all__ = ("SystemState",)

from abc import ABC, abstractmethod
from typing import AbstractSet, Collection, Mapping


class SystemState(ABC):
    """
    Provides a description of the instantaneous state of a ROS system in
    terms of its publishers, subscribers, and services.

    Attributes
    ----------
    publishers: Mapping[str, Collection[str]]
        A mapping from topics to the names of publishers to that topic.
    subscribers: Mapping[str, Collection[str]]
        A mapping from topics to the names of subscribers to that topic.
    services: Mapping[str, Collection[str]]
        A mapping from services to the names of providers of that service.
    nodes: AbstractSet[str]
        The names of all known nodes running on the system.
    topics: AbstractSet[str]
        The names of all known topics on the system with at least one
        publisher or one subscriber.
    """

    @property
    @abstractmethod
    def publishers(self) -> Mapping[str, Collection[str]]:
        ...

    @property
    @abstractmethod
    def subscribers(self) -> Mapping[str, Collection[str]]:
        ...

    @property
    @abstractmethod
    def services(self) -> Mapping[str, Collection[str]]:
        ...

    @property
    @abstractmethod
    def nodes(self) -> AbstractSet[str]:
        ...

    @property
    @abstractmethod
    def topics(self) -> AbstractSet[str]:
        ...
