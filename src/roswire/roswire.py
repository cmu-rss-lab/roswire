# -*- coding: utf-8 -*-
"""
This module provides access to the ROSWire session.
"""
__all__ = ('ROSWire',)

from typing import Dict, Iterator, Optional, Sequence
from uuid import uuid4
import os
import pathlib
import logging
import contextlib
import shutil

from docker import DockerClient
from docker import APIClient as DockerAPIClient

from .exceptions import ROSWireException
from .description import SystemDescription, SystemDescriptionManager
from .system import System
from .proxy import ContainerProxy, ContainerProxyManager
from .definitions import FormatDatabase, PackageDatabase, TypeDatabase

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ROSWire:
    """
    Provides an interface for building, analysing, and interacting with
    containerised ROS applications.

    Attributes
    ----------
    containers: ContainerProxyManager
        A manager for building and connecting to Docker containers.
    descriptions: SystemDescriptionManager
        A manager for building, loading, and storing static descriptions of
        ROS applications.
    workspace: str
        The absolute path of the workspace directory for this session. The
        workspace is used to store cache data and to store shared temporary
        directories between the host machine and containerised ROS
        applications.
    """
    def __init__(self,
                 dir_workspace: Optional[str] = None
                 ) -> None:
        if not dir_workspace:
            logger.debug("no workspace specified: using default workspace.")
            dir_home = os.path.expanduser("~")
            dir_workspace = os.path.join(dir_home, ".roswire")
            logger.debug("default workspace: %s", dir_workspace)
            if not os.path.exists(dir_workspace):
                logger.debug("initialising default workspace")
                os.mkdir(dir_workspace)
        else:
            logger.debug("using specified workspace: %s", dir_workspace)
            if not os.path.exists(dir_workspace):
                m = "workspace not found: {}".format(dir_workspace)
                raise ROSWireException(m)

        self.__dir_workspace = os.path.abspath(dir_workspace)
        self.__containers = ContainerProxyManager(self.__dir_workspace)
        dir_descriptions = os.path.join(dir_workspace, 'descriptions')
        self.__descriptions = SystemDescriptionManager(self.__containers,
                                                       dir_descriptions)

    @property
    def workspace(self) -> str:
        return self.__dir_workspace

    @property
    def client_docker(self) -> DockerClient:
        return self.__containers.docker_client

    @property
    def containers(self) -> ContainerProxyManager:
        return self.__containers

    @property
    def descriptions(self) -> SystemDescriptionManager:
        return self.__descriptions

    @contextlib.contextmanager
    def launch(self,
               image: str,
               sources: Sequence[str],
               description: Optional[SystemDescription] = None,
               *,
               ports: Optional[Dict[int, int]] = None
               ) -> Iterator[System]:
        """Launches a ROS application using a provided Docker image.

        Parameters
        ----------
        image: str
            the name of the Docker image.
        sources: Sequence[str]
            The sequence of setup files that should be used to load the ROS
            workspace.
        description: Optional[SystemDescription]
            an optional static description of the ROS application.
            If no description is provided, ROSWire will attempt to load one
            from the cache or else build one.
        ports: Dict[int, int], optional
            an optional dictionary specifying port mappings between the host
            and container, where keys represent container ports and values
            represent host ports.
        """
        if not description:
            description = self.descriptions.load_or_build(image, sources)
        with self.containers.launch(image,
                                    ports=ports,
                                    sources=sources) as container:
            container = container
            yield System(container, description)
