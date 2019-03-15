__all__ = ('ContainerProxy', )

from uuid import UUID

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer

from .file import FileProxy
from .shell import ShellProxy
from ..exceptions import RozzyException


class ContainerProxy:
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 uuid: UUID,
                 ws_host: str
                 ) -> None:
        self.__uuid = uuid
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo
        self.__ws_host = ws_host
        self.__shell = ShellProxy(daemon_bugzoo, container_bugzoo)
        self.__files = FileProxy(daemon_bugzoo,
                                 container_bugzoo,
                                 self.__ws_host,
                                 self.__shell)

    @property
    def uuid(self) -> UUID:
        return self.__uuid

    @property
    def shell(self) -> ShellProxy:
        return self.__shell

    @property
    def files(self) -> FileProxy:
        return self.__files

    @property
    def ws_host(self) -> str:
        return self.__ws_host

    @property
    def ip_address(self) -> str:
        bz = self.__daemon_bugzoo
        ip = bz.containers.ip_address(self.__container_bugzoo)
        if not ip:
            m = "no IP address for container: {}"
            m = m.format(self.uuid.hex)
            raise RozzyException(m)
        return str(ip)
