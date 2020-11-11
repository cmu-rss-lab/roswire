# -*- coding: utf-8 -*-
__all__ = ("SystemState",)

from typing import AbstractSet, Collection, Mapping, Set

import attr


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SystemState:
    """Provides a description of the instantaneous state of a ROS system in
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

    publishers: Mapping[str, Collection[str]]
    subscribers: Mapping[str, Collection[str]]
    services: Mapping[str, Collection[str]]
    nodes: AbstractSet[str] = attr.ib(init=False, repr=False)
    topics: AbstractSet[str] = attr.ib(init=False, repr=False)

    def __attrs_post_init__(self) -> None:
        nodes: Set[str] = set()
        nodes = nodes.union(*self.publishers.values())
        nodes = nodes.union(*self.subscribers.values())
        nodes = nodes.union(*self.services.values())

        topics: Set[str] = set()
        topics = topics.union(self.publishers)
        topics = topics.union(self.subscribers)

        object.__setattr__(self, "nodes", frozenset(nodes))
        object.__setattr__(self, "topics", frozenset(topics))
