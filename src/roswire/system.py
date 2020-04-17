# -*- coding: utf-8 -*-
__all__ = ('System',)

from typing import Iterator
from uuid import UUID
import contextlib
import logging

import attr
import dockerblade

from .description import SystemDescription
from .definitions import TypeDatabase
from .proxy import (ROSCore, ContainerProxy,
                    CatkinInterface, CatkinTools, CatkinMake)

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


@attr.s(frozen=True)
class System:
    """Provides access to a ROS application hosted by a Docker container.

    Attributes
    ----------
    uuid: UUID
        A unique identifier for the underlying container.
    description: SystemDescription
        A static description of the associated ROS application.
    messages: TypeDatabase
        A database of message types for the associated ROS application.
    shell: dockerblade.shell.Shell
        Provides access to a bash shell for this container.
    files: dockerblade.files.FileSystem
        Provides access to the filesystem for this container.
    ws_host: str
        The absolute path to the shared directory for this container's
        workspace on the host machine.
    container: ContainerProxy
        Provides access to the underlying Docker container.
    """
    container: ContainerProxy = attr.ib()
    description: SystemDescription = attr.ib()

    @property
    def uuid(self) -> UUID:
        return self.container.uuid

    @property
    def ws_host(self) -> str:
        return self.container.ws_host

    @property
    def ip_address(self) -> str:
        return self.container.ip_address

    @property
    def shell(self) -> dockerblade.shell.Shell:
        return self.container.shell

    @property
    def messages(self) -> TypeDatabase:
        return self.description.types

    def catkin(self, directory: str) -> CatkinInterface:
        """Returns an interface to a catkin workspace.

        Parameters
        ----------
        directory: str
            The absolute path to the catkin workspace inside the container.

        Returns
        -------
        CatkinInterface
            An interface to the given workspace.
        """
        # TODO decide whether to use catkin_tools or catkin_make
        return self.catkin_tools(directory)

    def catkin_tools(self, directory: str) -> CatkinTools:
        """Returns an interface to a catkin tools workspace.

        Parameters
        ----------
        directory: str
            The absolute path to the catkin workspace inside the container.

        Returns
        -------
        CatkinTools
            An interface to the given workspace.
        """
        return CatkinTools(shell=self.shell, directory=directory)

    def catkin_make(self, directory: str) -> CatkinMake:
        """Returns an interface to a catkin_make workspace.

        Parameters
        ----------
        directory: str
            The absolute path to the catkin workspace inside the container.

        Returns
        -------
        CatkinMake
            An interface to the given workspace.
        """
        return CatkinMake(shell=self.shell, directory=directory)

    @property
    def files(self) -> dockerblade.files.FileSystem:
        return self.container.files

    @contextlib.contextmanager
    def roscore(self, port: int = 11311) -> Iterator[ROSCore]:
        """
        Launches a context-managed roscore inside the container.
        Upon exiting the context, the ROS master (and its associated resources)
        will be destroyed.

        Parameters
        ----------
        port: int, optional
            The port inside the container on which roscore should run.

        Yields
        ------
        ROSCore
            An interface to the launched ROS Master.
        """
        assert port > 1023
        command = f"roscore -p {port}"
        process = self.shell.popen(command)
        try:
            yield ROSCore(description=self.description,
                          shell=self.shell,
                          files=self.files,
                          ws_host=self.ws_host,
                          ip_address=self.ip_address,
                          port=port)
        finally:
            process.terminate()
            process.wait(2.0)
            process.kill()
