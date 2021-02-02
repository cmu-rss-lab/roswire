# -*- coding: utf-8 -*-
__all__ = ("FormatDatabase",)

from abc import ABC, abstractmethod
from typing import Any, Dict, Generic, Mapping, TypeVar

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
    @abstractmethod
    def build(cls, db: PackageDatabase) -> "FormatDatabase":
        ...

    @property
    @abstractmethod
    def messages(self) -> Mapping[str, MF]:
        ...

    @property
    @abstractmethod
    def services(self) -> Mapping[str, SF]:
        ...

    @property
    @abstractmethod
    def actions(self) -> Mapping[str, AF]:
        ...

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
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        ...

    @classmethod
    def load(cls, fn: str) -> "FormatDatabase":
        ...
