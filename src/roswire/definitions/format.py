__all__ = ('FormatDatabase',)

from types import MappingProxyType
from typing import Collection, Mapping, Dict, Set

from .msg import MsgFormat
from .srv import SrvFormat
from .action import ActionFormat
from .package import Package, PackageDatabase


class FormatDatabase:
    @staticmethod
    def build(db: PackageDatabase) -> 'FormatDatabase':
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

    def __init__(self,
                 messages: Set[MsgFormat],
                 services: Set[SrvFormat],
                 actions: Set[ActionFormat]
                 ) -> None:
        self.__messages: Mapping[str, MsgFormat] = MappingProxyType({
            f.fullname: f for f in messages})
        self.__services: Mapping[str, SrvFormat] = MappingProxyType({
            f.fullname: f for f in services})
        self.__actions: Mapping[str, ActionFormat] = MappingProxyType({
            f.fullname: f for f in actions})

    @property
    def messages(self) -> Mapping[str, MsgFormat]:
        return self.__messages

    @property
    def services(self) -> Mapping[str, SrvFormat]:
        return self.__services

    @property
    def actions(self) -> Mapping[str, ActionFormat]:
        return self.__actions
