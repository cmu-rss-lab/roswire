__all__ = ('SystemInstance', )

from typing import Iterator
from uuid import UUID
import contextlib

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer

from .exceptions import RozzyException
from .proxy import ShellProxy, ROSProxy, FileProxy


class Container(object):
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 uuid: UUID,
                 ws_host: str
                 ) -> None:
        self.__daemon_bugzoo: BugZooDaemon = daemon_bugzoo
        self.__container_bugzoo: BugZooContainer = container_bugzoo
        self.__uuid: UUID = uuid
        self.__shell: ShellProxy = ShellProxy(daemon_bugzoo, container_bugzoo)
        self.__files: FileProxy = FileProxy(daemon_bugzoo,
                                            container_bugzoo,
                                            ws_host,
                                            self.__shell)
        self.__ws_host: str = ws_host

    @property
    def uuid(self) -> UUID:
        return self.__uuid

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

    @property
    def shell(self) -> ShellProxy:
        return self.__shell

    @property
    def files(self) -> FileProxy:
        return self.__files

    @contextlib.contextmanager
    def roscore(self, port: int = 11311) -> Iterator[ROSProxy]:
        assert port > 1023
        cmd = "roscore -p {}".format(port)
        self.shell.non_blocking_execute(cmd)
        try:
            yield ROSProxy(shell=self.shell,
                           ws_host=self.ws_host,
                           ip_address=self.ip_address,
                           port=port)
        finally:
            self.shell.execute("pkill roscore")
