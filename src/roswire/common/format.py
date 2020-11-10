# -*- coding: utf-8 -*-
__all__ = ("FormatDatabase",)

from types import MappingProxyType
from typing import Any, Dict, Mapping, Set

import yaml

from .action import ActionFormat
from .msg import MsgFormat
from .package import PackageDatabase
from .srv import SrvFormat


class FormatDatabase:
    """
    An immutable database of ROS definitions that maintains the parsed
    contents of :code:`.msg`, :code:`.srv`, and :code:`.action` files
    for messages, services, and action definitions.
    Note that implicit message definitions, i.e., those associated with a
    service or action definition, are also represented by the database.

    Attributes
    ----------
    messages: Mapping[str, MsgFormat]
        An immutable mapping from message name to its definition.
    services: Mapping[str, SrvFormat]
        An immutable mapping from service name to definition.
    actions: Mapping[str, ActionFormat]
        An immutable mapping from action name to definition.
    """

    @staticmethod
    def build(db: PackageDatabase) -> "FormatDatabase":
        """Constructs a format database from a given package database."""
        messages: Set[MsgFormat] = set()
        services: Set[SrvFormat] = set()
        actions: Set[ActionFormat] = set()

        for package in db.values():
            messages.update(package.messages)
            services.update(package.services)
            actions.update(package.actions)

            for service in package.services:
                if service.request:
                    messages.add(service.request)
                if service.response:
                    messages.add(service.response)

            for action in package.actions:
                if action.goal:
                    messages.add(action.goal)
                if action.result:
                    messages.add(action.result)
                if action.feedback:
                    messages.add(action.feedback)

        return FormatDatabase(messages, services, actions)

    def __init__(
        self,
        messages: Set[MsgFormat],
        services: Set[SrvFormat],
        actions: Set[ActionFormat],
    ) -> None:
        self.__messages: Mapping[str, MsgFormat] = MappingProxyType(
            {f.fullname: f for f in messages}
        )
        self.__services: Mapping[str, SrvFormat] = MappingProxyType(
            {f.fullname: f for f in services}
        )
        self.__actions: Mapping[str, ActionFormat] = MappingProxyType(
            {f.fullname: f for f in actions}
        )

    @property
    def messages(self) -> Mapping[str, MsgFormat]:
        return self.__messages

    @property
    def services(self) -> Mapping[str, SrvFormat]:
        return self.__services

    @property
    def actions(self) -> Mapping[str, ActionFormat]:
        return self.__actions

    def to_dict(self) -> Dict[str, Any]:
        """Returns a JSON description of this database."""
        return {
            "messages": [m.to_dict() for m in self.__messages.values()],
            "services": [s.to_dict() for s in self.__services.values()],
            "actions": [a.to_dict() for a in self.__actions.values()],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ActionFormat.from_dict(dd) for dd in d["actions"]}
        return FormatDatabase(msg, srv, action)

    def save(self, fn: str) -> None:
        """Saves the contents of this format database to disk."""
        with open(fn, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    @classmethod
    def load(cls, fn: str) -> "FormatDatabase":
        """Loads a format database from a given file on disk."""
        with open(fn, "r") as f:
            return cls.from_dict(yaml.safe_load(f))
