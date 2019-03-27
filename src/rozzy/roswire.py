# -*- coding: utf-8 -*-
"""
This module provides access to the ROSWire session.
"""
__all__ = ('ROSWire',)

from typing import Optional, Dict, Iterator
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
        self.__client_docker = DockerClient()
        self.__api_docker = \
            DockerAPIClient(base_url='unix://var/run/docker.sock')
        self.__containers = ContainerProxyManager(self.__client_docker,
                                                  self.__api_docker,
                                                  self.__dir_workspace)
        dir_descriptions = os.path.join(dir_workspace, 'descriptions')
        self.__descriptions = SystemDescriptionManager(self.__containers,
                                                       dir_descriptions)

    @property
    def workspace(self) -> str:
        return self.__dir_workspace

    @property
    def client_docker(self) -> DockerClient:
        return self.__client_docker

    @property
    def containers(self) -> ContainerProxyManager:
        return self.__containers

    @property
    def descriptions(self) -> SystemDescriptionManager:
        return self.__descriptions

    @contextlib.contextmanager
    def launch(self,
               image: str,
               description: Optional[SystemDescription] = None
               ) -> Iterator[System]:
        """
        Launches a ROS application using a provided Docker image.

        Parameters
        ----------
            image: str
                the name of the Docker image.
            description: Optional[SystemDescription]
                an optional static description of the ROS application.
                If no description is provided, ROSWire will attempt to load one
                from the cache or else build one.
        """
        if not description:
            description = self.descriptions.build(image)
        with self.containers.launch(image) as container:
            container = container
            yield System(container, description)
