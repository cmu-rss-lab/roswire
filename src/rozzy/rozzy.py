__all__ = ('Rozzy',)

from typing import Optional, Dict, Iterator
from uuid import uuid4
import os
import pathlib
import logging
import contextlib
import shutil

from docker import DockerClient

from .exceptions import RozzyException
from .system import System, SystemDescription
from .proxy import ContainerProxy
from .definitions import FormatDatabase, PackageDatabase, TypeDatabase

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class Rozzy:
    def __init__(self,
                 dir_workspace: Optional[str] = None
                 ) -> None:
        if not dir_workspace:
            logger.debug("no workspace specified: using default workspace.")
            dir_home = os.path.expanduser("~")
            dir_workspace = os.path.join(dir_home, ".rozzy")
            logger.debug("default workspace: %s", dir_workspace)
            if not os.path.exists(dir_workspace):
                logger.debug("initialising default workspace")
                os.mkdir(dir_workspace)
        else:
            logger.debug("using specified workspace: %s", dir_workspace)
            if not os.path.exists(dir_workspace):
                m = "workspace not found: {}".format(dir_workspace)
                raise RozzyException(m)

        self.__dir_workspace = os.path.abspath(dir_workspace)
        self.__client_docker = DockerClient()

    @property
    def workspace(self) -> str:
        """
        The absolute path to the workspace directory.
        """
        return self.__dir_workspace

    @property
    def client_docker(self) -> DockerClient:
        return self.__client_docker

    def describe(self, image: str) -> SystemDescription:
        """
        Loads a description of the system provided by a given Docker image.
        """
        args = [self.client_docker, self.workspace, image]
        with ContainerProxy.launch(*args) as container:
            paths = PackageDatabase.paths(container.shell)
            db_package = PackageDatabase.from_paths(container.files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)
        return SystemDescription(image=image,
                                 packages=db_package,
                                 formats=db_format,
                                 types=db_type)

    @contextlib.contextmanager
    def launch(self, desc: SystemDescription) -> Iterator[System]:
        args = [self.client_docker, self.workspace, desc.image]
        with ContainerProxy.launch(*args) as container:
            container = container
            yield System(container)
