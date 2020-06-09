# -*- coding: utf-8 -*-
"""
This module provides access to the ROSWire session.
"""
__all__ = ('ROSWire',)

from typing import Dict, Iterator, Mapping, Optional, Sequence
import os
import contextlib

from loguru import logger

from .exceptions import ROSWireException
from .description import SystemDescription, SystemDescriptionManager
from .system import System
from .proxy import ContainerManager


class ROSWire:
    """
    Provides an interface for building, analysing, and interacting with
    containerised ROS applications.

    Attributes
    ----------
    containers: ContainerManager
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
            logger.debug(f"default workspace: {dir_workspace}")
            if not os.path.exists(dir_workspace):
                logger.debug("initialising default workspace")
                os.mkdir(dir_workspace)
        else:
            logger.debug(f"using specified workspace: {dir_workspace}")
            if not os.path.exists(dir_workspace):
                m = f"workspace not found: {dir_workspace}"
                raise ROSWireException(m)

        self.__dir_workspace = os.path.abspath(dir_workspace)
        self.__containers = ContainerManager(self.__dir_workspace)
        dir_descriptions = os.path.join(dir_workspace, 'descriptions')
        self.__descriptions = SystemDescriptionManager(self.__containers,
                                                       dir_descriptions)

    @property
    def workspace(self) -> str:
        return self.__dir_workspace

    @property
    def containers(self) -> ContainerManager:
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
               ports: Optional[Dict[int, int]] = None,
               environment: Optional[Mapping[str, str]] = None,
               network_mode: str = 'bridge'
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
        environment: Mapping[str, str], optional
            an optional set of additional environment variables, indexed by
            name, that should be used by the system.
        network_mode: str
            The Docker network mode that should be used by the container. This
            may be `bridge`, `host`, `none`, or `container:<name|id>`. Note
            that OSX does not provide full support for `bridge` mode.
        """
        if not description:
            description = self.descriptions.load_or_build(image, sources)
        with self.containers.launch(image,
                                    ports=ports,
                                    sources=sources,
                                    network_mode=network_mode,
                                    environment=environment) as container:
            container = container
            yield System(container, description)
