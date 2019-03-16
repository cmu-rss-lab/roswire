__all__ = ('SystemDescription', 'SystemDescriptionManager', 'System')

from typing import Iterator, Union
from uuid import UUID
import contextlib

import attr
from docker import DockerClient
from docker.models.images import Image as DockerImage

from .exceptions import RozzyException
from .definitions import TypeDatabase, FormatDatabase, PackageDatabase
from .proxy import (ShellProxy, ROSProxy, FileProxy, ContainerProxy,
                    ContainerProxyManager)


@attr.s
class SystemDescription:
    sha256: str = attr.ib()
    types: TypeDatabase = attr.ib()
    formats: FormatDatabase = attr.ib()
    packages: PackageDatabase = attr.ib()


class SystemDescriptionManager:
    def __init__(self,
                 containers: ContainerProxyManager
                 ) -> None:
        self.__containers = containers

    def build(self,
              image_or_tag: Union[str, DockerImage]
              ) -> SystemDescription:
        image: DockerImage
        if isinstance(image_or_tag, str):
            image = self.__containers.image(image_or_tag)
        else:
            image = image_or_tag

        sha256: str = image.id[7:]
        with self.__containers.launch(image) as container:
            paths = PackageDatabase.paths(container.shell)
            db_package = PackageDatabase.from_paths(container.files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)
        return SystemDescription(sha256=sha256,
                                 packages=db_package,
                                 formats=db_format,
                                 types=db_type)


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
