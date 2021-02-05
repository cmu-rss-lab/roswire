# -*- coding: utf-8 -*-
__all__ = ("FormatDatabase",)

from abc import ABC, abstractmethod
from types import MappingProxyType
from typing import Any, Dict, Generic, Mapping, Set, TypeVar

import yaml

from .action import ActionFormat
from .msg import MsgFormat
from .package import PackageDatabase
from .srv import SrvFormat

MF = TypeVar("MF", bound=MsgFormat)
SF = TypeVar("SF", bound=SrvFormat)
AF = TypeVar("AF", bound=ActionFormat)


class FormatDatabase(ABC, Generic[MF, SF, AF]):
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

    @classmethod
    def from_packages(cls,
                      db: PackageDatabase
                      ) -> "FormatDatabase[MF, SF, AF]":
        """Constructs a format database from a given package database."""
        messages: Set[MF] = set()
        services: Set[SF] = set()
        actions: Set[AF] = set()

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

        return cls(messages, services, actions)

    def __init__(
        self,
        messages: Set[MF],
        services: Set[SF],
        actions: Set[AF],
    ) -> None:
        self.__messages: Mapping[str, MF] = MappingProxyType(
            {f.fullname: f for f in messages}
        )
        self.__services: Mapping[str, SF] = MappingProxyType(
            {f.fullname: f for f in services}
        )
        self.__actions: Mapping[str, AF] = MappingProxyType(
            {f.fullname: f for f in actions}
        )

    @property
    def messages(self) -> Mapping[str, MF]:
        return self.__messages

    @property
    def services(self) -> Mapping[str, SF]:
        return self.__services

    @property
    def actions(self) -> Mapping[str, AF]:
        return self.__actions

    def to_dict(self) -> Dict[str, Any]:
        """Returns a JSON description of this database."""
        return {
            "messages": [m.to_dict() for m in self.messages.values()],
            "services": [s.to_dict() for s in self.services.values()],
            "actions": [a.to_dict() for a in self.actions.values()],
        }

    def save(self, fn: str) -> None:
        """Saves the contents of this format database to disk."""
        with open(fn, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    @classmethod
    @abstractmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        ...

    @classmethod
    def load(cls, fn: str) -> "FormatDatabase":
        """Loads a format database from a given file on disk."""
        with open(fn, "r") as f:
            return cls.from_dict(yaml.safe_load(f))
