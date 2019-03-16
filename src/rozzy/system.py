__all__ = ('SystemDescription', 'System')

from typing import Iterator
from uuid import UUID
import contextlib

import attr

from .exceptions import RozzyException
from .definitions import TypeDatabase, FormatDatabase, PackageDatabase
from .proxy import ShellProxy, ROSProxy, FileProxy, ContainerProxy


@attr.s
class SystemDescription:
    sha256: str = attr.ib()
    types: TypeDatabase = attr.ib()
    formats: FormatDatabase = attr.ib()
    packages: PackageDatabase = attr.ib()


class System:
    def __init__(self,
                 container: ContainerProxy,
                 description: SystemDescription
                 ) -> None:
        self.__container = container
        self.__description = description

    @property
    def description(self) -> SystemDescription:
        return self.__description

    @property
    def uuid(self) -> UUID:
        return self.__container.uuid

    @property
    def ws_host(self) -> str:
        return self.__container.ws_host

    @property
    def ip_address(self) -> str:
        return self.__container.ip_address

    @property
    def shell(self) -> ShellProxy:
        return self.__container.shell

    @property
    def files(self) -> FileProxy:
        return self.__container.files

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
