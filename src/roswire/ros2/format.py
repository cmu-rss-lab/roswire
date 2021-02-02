# -*- coding: utf-8 -*-
__all__ = ("ROS2FormatDatabase",)

from types import MappingProxyType
from typing import Any, Dict, Mapping, Set

import yaml

from ..common import (
    ActionFormat,
    FormatDatabase,
    MsgFormat,
    PackageDatabase,
    SrvFormat,
)


class ROS2FormatDatabase(FormatDatabase[MsgFormat, SrvFormat, ActionFormat]):

    @classmethod
    def build(cls, db: PackageDatabase) -> "ROS2FormatDatabase":
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

        return ROS2FormatDatabase(messages, services, actions)

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

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "FormatDatabase":
        """Loads a format database from a JSON document."""
        msg = {MsgFormat.from_dict(dd) for dd in d["messages"]}
        srv = {SrvFormat.from_dict(dd) for dd in d["services"]}
        action = {ActionFormat.from_dict(dd) for dd in d["actions"]}
        return ROS2FormatDatabase(msg, srv, action)

    @classmethod
    def load(cls, fn: str) -> "FormatDatabase":
        """Loads a format database from a given file on disk."""
        with open(fn, "r") as f:
            return cls.from_dict(yaml.safe_load(f))
