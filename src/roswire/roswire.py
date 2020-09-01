# -*- coding: utf-8 -*-
"""
This module provides access to the ROSWire session.
"""
__all__ = ('ROSWire',)

import contextlib
import os
from typing import Dict, Iterator, Mapping, Optional, Sequence

import dockerblade
import yaml
from loguru import logger

from .app import App, AppDescription, AppInstance
from .exceptions import ROSWireException

_DEFAULT_URL = os.environ.get('DOCKER_HOST', 'unix://var/run/docker.sock')


class ROSWire:
    """
    Provides an interface for building, analysing, and interacting with
    containerised ROS applications.

    Attributes
    ----------
    workspace: str
        The absolute path of the workspace directory for this session. The
        workspace is used to store cache data and to store shared temporary
        directories between the host machine and containerised ROS
        applications.
    """
    def __init__(self,
                 *,
                 workspace: Optional[str] = None,
                 docker_url: str = _DEFAULT_URL
                 ) -> None:
        if not workspace:
            logger.debug("no workspace specified: using default workspace.")
            dir_home = os.path.expanduser("~")
            workspace = os.path.join(dir_home, ".roswire")
            logger.debug(f"default workspace: {workspace}")
            if not os.path.exists(workspace):
                logger.debug("initialising default workspace")
                os.mkdir(workspace)
        else:
            logger.debug(f"using specified workspace: {workspace}")
            if not os.path.exists(workspace):
                m = f"workspace not found: {workspace}"
                raise ROSWireException(m)

        self.__workspace = os.path.abspath(workspace)
        self._dockerblade = dockerblade.DockerDaemon(docker_url)

    def __repr__(self) -> str:
        return f"ROSWire(workspace='{self.workspace}')"

    @property
    def workspace(self) -> str:
        return self.__workspace

    def app(self, image: str, sources: Sequence[str]) -> App:
        """Constructs a ROS application."""
        return App(image=image, sources=sources, roswire=self)

    def load(self, filename: str) -> App:
        """Loads a ROS application from a given file."""
        with open(filename, 'r') as f:
            contents = yaml.safe_load(f)
        image: str = contents['image']
        sources: Sequence[str] = contents['sources']
        app = self.app(image=image, sources=sources)

        if 'description' in contents:
            description = \
                AppDescription._from_dict_for_app(contents['description'], app)
            object.__setattr__(app, '_description', description)

        return app

    @contextlib.contextmanager
    def launch(self,
               image: str,
               sources: Sequence[str],
               description: Optional[AppDescription] = None,
               *,
               ports: Optional[Dict[int, int]] = None,
               environment: Optional[Mapping[str, str]] = None
               ) -> Iterator[AppInstance]:
        """Launches a ROS application using a provided Docker image.

        Parameters
        ----------
        image: str
            the name of the Docker image.
        sources: Sequence[str]
            The sequence of setup files that should be used to load the ROS
            workspace.
        description: Optional[AppDescription]
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
        """
        app: App
        if description:
            app = description.app
        else:
            app = self.app(image=image, sources=sources)
        with app.launch(ports=ports, environment=environment) as app_instance:
            yield app_instance
