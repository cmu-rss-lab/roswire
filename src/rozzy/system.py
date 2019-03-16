__all__ = ('SystemDescription', 'SystemDescriptionManager', 'System')

from typing import Iterator, Union, Dict, Any
from uuid import UUID
import contextlib
import logging
import os

import yaml
import attr
from docker import DockerClient
from docker.models.images import Image as DockerImage

from .exceptions import RozzyException
from .definitions import TypeDatabase, FormatDatabase, PackageDatabase
from .proxy import (ShellProxy, ROSProxy, FileProxy, ContainerProxy,
                    ContainerProxyManager)

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(loging.DEBUG)


@attr.s(slots=True)
class SystemDescription:
    sha256: str = attr.ib()
    types: TypeDatabase = attr.ib()
    formats: FormatDatabase = attr.ib()
    packages: PackageDatabase = attr.ib()

    @staticmethod
    def from_file(fn: str) -> 'SystemDescription':
        with open(fn, 'r') as f:
            d = yaml.load(f)
        return SystemDescription.from_dict(d)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'SystemDescription':
        sha256: str = d['sha256']
        packages = PackageDatabase.from_dict(d['packages'])
        formats = FormatDatabase.build(packages)
        types = TypeDatabase.build(formats)
        return SystemDescription(sha256=sha256,
                                 packages=packages,
                                 formats=formats,
                                 types=types)

    def to_dict(self) -> Dict[str, Any]:
        return {'sha256': self.sha256,
                'packages': self.packages.to_dict()}


class SystemDescriptionManager:
    def __init__(self,
                 containers: ContainerProxyManager,
                 dir_cache: str
                 ) -> None:
        self.__containers = containers

        # ensure that the cache directory exists
        self.__dir_cache = dir_cache
        os.makedirs(dir_cache, exist_ok=True)

    def load(self,
             image_or_tag: Union[str, DockerImage]
             ) -> SystemDescription:
        sha256 = self.__containers.image_sha256(image_or_tag)
        fn = os.path.join(self.__dir_cache, sha256)
        try:
            return  SystemDescription.from_file(fn)
        except FileNotFoundError:
            logger.exception("failed to load description for image: %s",
                             image_or_tag)
            raise

    def save(self, description: SystemDescription) -> None:
        fn = os.path.join(self.__dir_cache, description.sha256)
        yml = description.to_dict()
        with open(fn, 'w') as f:
            yaml.dump(yml, f, default_flow_style=False)

    def build(self,
              image_or_tag: Union[str, DockerImage],
              save: bool = True
              ) -> SystemDescription:
        sha256 = self.__containers.image_sha256(image_or_tag)
        with self.__containers.launch(image_or_tag) as container:
            paths = PackageDatabase.paths(container.shell)
            db_package = PackageDatabase.from_paths(container.files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)

        description = SystemDescription(sha256=sha256,
                                        packages=db_package,
                                        formats=db_format,
                                        types=db_type)
        if save:
            self.save(description)
        return description


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
