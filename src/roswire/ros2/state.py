# -*- coding: utf-8 -*-
__all__ = ("ROS2StateProbe",)

import typing
from typing import AbstractSet, Collection, Dict, Mapping, Optional, Set

import attr
import dockerblade
from loguru import logger

from ..common import SystemState
from ..exceptions import ConflictingTypes

_ACTION_CLIENTS = "act_cli"
_ACTION_SERVERS = "act_serv"
_SERVICE_CLIENTS = "cli"
_SERVICES = "serv"
_SUBSCRIBERS = "sub"
_PUBLISHERS = "pub"

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2SystemState(SystemState):
    """
    Provides a description of the instantaneous state of a ROS2 system in
    terms of its publishers, subscribers, actions, and services.

    Attributes
    ----------
    publishers: Mapping[str, Collection[str]]
        A mapping from topics to the names of publishers to that topic.
    subscribers: Mapping[str, Collection[str]]
        A mapping from topics to the names of subscribers to that topic.
    services: Mapping[str, Collection[str]]
        A mapping from services to the names of providers of that service.
    clients: Mapping[str, Collection[str]]
        A mapping from services to the names of clients of that service.
    action_servers: Mapping[str, Collection[str]]
        A mapping from actions to the names of providesr of that action.
    action_clients: Mapping[str, Collection[str]]
        A mapping from actions to the names of clients of that action
    topic_to_type: Mapping[str, str]
        A mapping from topics to the name of their type
    service_to_type: Mapping[str, str]
        A mapping from services to the name of their type
    action_to_type: Mapping[str, str]
        A mapping from actions to the name of their type
    nodes: AbstractSet[str]
        The names of all known nodes running on the system.
    topics: AbstractSet[str]
        The names of all known topics on the system with at least one
        publisher or one subscriber.
    service_names: AbstractSet[str]
        The name of all the known services on the system with at least
        one server or client
    action_names: AbstractSet[str]
        The name of all the known actions on the system with at least one
        one action server or client
    """

    publishers: Mapping[str, Collection[str]]
    subscribers: Mapping[str, Collection[str]]
    services: Mapping[str, Collection[str]]
    service_clients: Mapping[str, Collection[str]]
    action_servers: Mapping[str, Collection[str]]
    action_clients: Mapping[str, Collection[str]]
    topic_to_type: Mapping[str, str]
    service_to_type: Mapping[str, str]
    action_to_type: Mapping[str, str]
    nodes: AbstractSet[str] = attr.ib(init=False, repr=False)
    topics: AbstractSet[str] = attr.ib(init=False, repr=False)
    service_names: AbstractSet[str] = attr.ib(init=False, repr=False)
    action_names: AbstractSet[str] = attr.ib(init=False, repr=False)

    def __attrs_post_init__(self) -> None:
        nodes: Set[str] = set()
        nodes = nodes.union(*self.publishers.values())
        nodes = nodes.union(*self.subscribers.values())
        nodes = nodes.union(*self.services.values())
        nodes = nodes.union(*self.service_clients.values())
        nodes = nodes.union(*self.action_servers.values())
        nodes = nodes.union(*self.action_clients.values())

        topics: Set[str] = set()
        topics = topics.union(self.publishers.keys())
        topics = topics.union(self.subscribers.keys())

        service_names: Set[str] = set()
        service_names = service_names.union(self.services.keys())
        service_names = service_names.union(self.service_clients.keys())

        action_names: Set[str] = set()
        action_names = action_names.union(self.action_servers.keys())
        action_names = action_names.union(self.action_clients.keys())

        object.__setattr__(self, "nodes", frozenset(nodes))
        object.__setattr__(self, "topics", frozenset(topics))
        object.__setattr__(self, "service_names", frozenset(service_names))
        object.__setattr__(self, "action_names", frozenset(action_names))


@attr.s(frozen=True, auto_attribs=True)
class ROS2StateProbe:
    """
    Provides an interface for obtaining the instantaneous state of a ROS
    system in terms of its publishers, subscribers, and services.
    """

    _app_instance: "AppInstance" = attr.ib()

    @classmethod
    def for_app_instance(cls, app_instance: "AppInstance") -> "ROS2StateProbe":
        return ROS2StateProbe(app_instance=app_instance)

    def probe(self) -> ROS2SystemState:
        """Obtains the instantaneous state of the associated ROS system.

        Raises
        ------
        ConflictingTypeException
            If more than one type is detected for a given topic, service,
            or action
        """
        shell = self._app_instance.shell
        node_to_state: Dict[Optional[str], Dict[str, Set[str]]] = {
            _PUBLISHERS: {},
            _SUBSCRIBERS: {},
            _SERVICES: {},
            _SERVICE_CLIENTS: {},
            _ACTION_SERVERS: {},
            _ACTION_CLIENTS: {}
        }

        topic_to_type: Dict[str, str] = {}
        service_to_type: Dict[str, str] = {}
        action_to_type: Dict[str, str] = {}

        command = "ros2 node list"
        try:
            output = shell.check_output(command, text=True)
        except dockerblade.exceptions.CalledProcessError:
            logger.debug("Unable to retrieve rosnode list from command line")
            raise
        node_names = output.split("\r\n")
        for node_name in node_names:
            if node_name == "":
                continue
            command_info = f"ros2 node info '{node_name}'"
            mode: Optional[str] = None
            types: Dict[str, str] = {}
            try:
                output = shell.check_output(command_info, text=True)
            except dockerblade.exceptions.CalledProcessError:
                logger.debug(f"Unable to retrieve '{command_info}'")
                raise
            output = output.replace(" ", "")
            # Uses mode to parse line and add info to appropriate set
            lines = output.split("\r\n")
            for line in lines:
                if "Publishers:" in line:
                    mode = _PUBLISHERS
                    types = topic_to_type
                    continue
                elif "Subscribers:" in line:
                    mode = _SUBSCRIBERS
                    types = topic_to_type
                    continue
                elif "Services:" in line:
                    mode = _SERVICES
                    types = service_to_type
                    continue
                elif "Action Servers:" in line:
                    mode = _ACTION_SERVERS
                    types = action_to_type
                    continue
                elif "Service Clients:" in line:
                    mode = _SERVICE_CLIENTS
                    types = service_to_type
                    continue
                elif "Action Clients:" in line:
                    mode = _ACTION_CLIENTS
                    types = action_to_type
                    continue

                if mode:
                    partition = line.partition(":")
                    name = partition[0]
                    fmt = partition[1]
                    if name in node_to_state[mode]:
                        node_to_state[mode][name].add(node_name)
                    else:
                        node_to_state[mode][name] = {node_name}
                    # Add type information and report conflict if a
                    # different types was registered (probably
                    # should never happen)
                    if name in types and fmt != types[name]:
                        logger.error(
                            f'The entity {name} has conflicting types: '
                            f'{types[name]} =/= {fmt}')
                        raise ConflictingTypes(entity=name,
                                               existing=types[name],
                                               conflicting=fmt)
                    else:
                        types[name] = fmt

        state = ROS2SystemState(
            publishers=node_to_state[_PUBLISHERS],
            subscribers=node_to_state[_SUBSCRIBERS],
            services=node_to_state[_SERVICES],
            service_clients=node_to_state[_SERVICE_CLIENTS],
            action_servers=node_to_state[_ACTION_SERVERS],
            action_clients=node_to_state[_ACTION_CLIENTS],
            topic_to_type=topic_to_type,
            service_to_type=service_to_type,
            action_to_type=action_to_type
        )
        return state

    __call__ = probe
