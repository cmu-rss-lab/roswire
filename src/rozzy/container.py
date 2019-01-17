__all__ = ['Container']

from uuid import UUID

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer


class Container(object):
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 uuid: UUID
                 ) -> None:
        self.__is_alive = True
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo
        self.__uuid = uuid

    @property
    def is_alive(self) -> bool:
        return self.__is_alive

    def _close(self) -> None:
        if self.__is_alive:
            del self.__daemon_bugzoo.containers[self.__container_bugzoo.id]
            self.__is_alive = False
