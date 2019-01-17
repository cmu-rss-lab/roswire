__all__ = ['Container', 'ShellProxy']

from typing import Optional, Tuple
from uuid import UUID

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer


class ShellProxy(object):
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo

    def execute(self,
                command: str,
                context: Optional[str] = None,
                stdout: bool = True,
                stderr: bool = False
                ) -> Tuple[int, str, float]:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo,
                        command,
                        context=context,
                        stdout=stdout,
                        stderr=stderr)
        return (r.code, r.output, r.duration)


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
        return ShellProxy(self.__daemon_bugzoo,
                          self.__container_bugzoo)
