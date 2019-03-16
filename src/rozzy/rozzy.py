__all__ = ('Rozzy',)

from typing import Optional, Dict, Iterator
from uuid import uuid4
import os
import pathlib
import logging
import contextlib
import shutil

from docker import DockerClient
from docker import APIClient as DockerAPIClient

from .exceptions import RozzyException
from .system import System, SystemDescription
from .proxy import ContainerProxy, ContainerProxyManager
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
        self.__api_docker = \
            DockerAPIClient(base_url='unix://var/run/docker.sock')
        self.__containers = ContainerProxyManager(self.__client_docker,
                                                  self.__api_docker,
                                                  self.__dir_workspace)

    @property
    def workspace(self) -> str:
        """
        The absolute path to the workspace directory.
        """
        return self.__dir_workspace

    @property
    def client_docker(self) -> DockerClient:
        return self.__client_docker

    @property
    def containers(self) -> ContainerProxyManager:
        return self.__containers

    @contextlib.contextmanager
    def launch(self,
               image: str,
               description: Optional[SystemDescription] = None
               ) -> Iterator[System]:
        if not description:
            description = SystemDescription.build(self.containers, image)
        with self.containers.launch(image) as container:
            container = container
            yield System(container, description)
