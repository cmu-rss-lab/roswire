# -*- coding: utf-8 -*-
__all__ = ('Topic', 'TopicManager')

from typing import Iterator, Mapping

import attr


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Topic:
    name: str


class TopicManager(Mapping[str, Topic]):
    """Used to provide information about and interact with ROS topics.
    Mimics the interface of `rostopic <http://wiki.ros.org/rostopic>`_."""
    def __init__(self,
                 description: SystemDescription,
                 host_ip_master: str,
                 api: xmlrpc.client.Server,
                 shell: dockerblade.Shell
                 ) -> None:
        self.__description = description
        self.__host_ip_master = host_ip_master
        self.__api = api
        self.__shell = shell

    def __iter__(self) -> Iterator[str]:
        """Returns an iterator over the names of all ROS topics that are
        advertised on the associated ROS Master."""
        raise NotImplementedError

    def __getitem__(self, name: str) -> Topic:
        """Retrieves information about a given topic."""
        raise NotImplementedError

    def __len__(self) -> int:
        """Returns the number of topics that are currently advertised."""
        raise NotImplementedError
