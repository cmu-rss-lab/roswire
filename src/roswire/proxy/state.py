# -*- coding: utf-8 -*-
__all__ = ('SystemState', 'SystemStateProbe')

import xmlrpc
from typing import AbstractSet, Collection, Dict, Mapping, Sequence, Set, Tuple

import attr

from .. import exceptions as exc


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

        object.__setattr__(self, 'nodes', frozenset(nodes))
        object.__setattr__(self, 'topics', frozenset(topics))


@attr.s(frozen=True, auto_attribs=True)
class SystemStateProbe:
    """Provides an interface for obtaining the instantaneous state of a ROS
    system in terms of its publishers, subscribers, and services."""
    _connection: xmlrpc.client.ServerProxy

    @classmethod
    def via_xmlrpc_connection(cls,
                              connection: xmlrpc.client.ServerProxy
                              ) -> 'SystemStateProbe':
        return SystemStateProbe(connection)

    def probe(self) -> SystemState:
        """Obtains the instantaneous state of the associated ROS system."""
        code: int
        msg: str
        result: Tuple[Sequence[Tuple[str, Sequence[str]]],
                      Sequence[Tuple[str, Sequence[str]]],
                      Sequence[Tuple[str, Sequence[str]]]]
        code, msg, result = \
            self._connection.getSystemState('roswire-probe')  # type: ignore
        if code != 1:
            raise exc.ROSWireException("probe failed!")

        publishers: Dict[str, Sequence[str]] = {}
        subscribers: Dict[str, Sequence[str]] = {}
        services: Dict[str, Sequence[str]] = {}

        for topic, publisher_names in result[0]:
            publishers[topic] = publisher_names
        for topic, subscriber_names in result[1]:
            subscribers[topic] = subscriber_names
        for topic, service_names in result[2]:
            services[topic] = service_names

        state = SystemState(publishers=publishers,
                            subscribers=subscribers,
                            services=services)
        return state

    __call__ = probe
