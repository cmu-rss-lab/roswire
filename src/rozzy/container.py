__all__ = ['Container', 'ShellProxy']

from typing import Optional, Tuple
from uuid import UUID

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer
from bugzoo.cmd import PendingExecResponse


class ShellProxy(object):
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo

    def execute(self, command: str, **kwargs) -> Tuple[int, str, float]:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo, command, **kwargs)
        return r.code, r.output, r.duration

    def non_blocking_execute(self,
                             command: str,
                             **kwargs
                             ) -> PendingExecResponse:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo, command, block=False, **kwargs)
        return r


class Container(object):
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 uuid: UUID
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo
        self.__uuid = uuid

    @property
    def shell(self) -> ShellProxy:
        return ShellProxy(self.__daemon_bugzoo, self.__container_bugzoo)
