# -*- coding: utf-8 -*-
__all__ = ("ROS2StateProbe",)

import typing
from typing import AbstractSet, Collection, Dict, Mapping, Optional, Set

import attr
import dockerblade
from loguru import logger

from ..common import SystemState

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(frozen=True, auto_attribs=True, slots=True)
class ROS2SystemState(SystemState):
    """
    Provides a description of the instantaneous state of a ROS2 system in
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
        """Obtains the instantaneous state of the associated ROS system."""
        shell = self._app_instance.shell
        node_to_state: Dict[Optional[str], Dict[str, Set[str]]] = {
            "pub": {},
            "sub": {},
            "serv": {},
        }
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
                    mode = "pub"
                    continue
                elif "Subscribers:" in line:
                    mode = "sub"
                    continue
                elif "Services:" in line:
                    mode = "serv"
                    continue
                elif "Action Servers:" in line:
                    break

                if mode:
                    name = line.partition(":")[0]
                    if name in node_to_state[mode]:
                        node_to_state[mode][name].add(node_name)
                    else:
                        node_to_state[mode][name] = {node_name}

        state = ROS2SystemState(
            publishers=node_to_state["pub"],
            subscribers=node_to_state["sub"],
            services=node_to_state["serv"],
        )
        return state

    __call__ = probe
