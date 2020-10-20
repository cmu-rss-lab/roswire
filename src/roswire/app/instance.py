# -*- coding: utf-8 -*-
__all__ = ('AppInstance',)

import contextlib
import os
import shutil
import typing
from types import TracebackType
from typing import Iterator, Optional, Type

import attr
import dockerblade
from docker.models.images import Image as DockerImage
from loguru import logger

from ..common import TypeDatabase
from ..common.catkin import CatkinInterface, CatkinMake, CatkinTools
from ..ros1.ros1 import ROS1
from ..ros2 import ROS2

if typing.TYPE_CHECKING:
    from .app import App
    from .description import AppDescription


@attr.s(frozen=True, slots=True)
class AppInstance:
    """Provides access to an instance of a ROS application.

    Attributes
    ----------
    app: App
        The associated ROS application.
    description: AppDescription
        A description of the associated ROS application.
    messages: TypeDatabase
        A database of message types for the associated ROS application.
    shell: dockerblade.shell.Shell
        Provides access to a bash shell for this container.
    files: dockerblade.files.FileSystem
        Provides access to the filesystem for this container.
    _host_workspace: str
        The absolute path to the shared directory for this container's
        workspace on the host machine.
    _dockerblade: dockerblade.container.Container
        Provides access to the underlying Docker container.
    """
    _dockerblade: dockerblade.container.Container = attr.ib()
    _host_workspace: str = attr.ib(repr=False)
    app: 'App' = attr.ib()
    shell: dockerblade.shell.Shell = attr.ib(repr=False, init=False, eq=False)
    files: dockerblade.files.FileSystem = \
        attr.ib(repr=False, init=False, eq=False)

    def __attrs_post_init__(self) -> None:
        dockerblade = self._dockerblade

        shell = dockerblade.shell('/bin/bash', sources=self.app.sources)
        object.__setattr__(self, 'shell', shell)

        files = dockerblade.filesystem()
        object.__setattr__(self, 'files', files)

    @property
    def description(self) -> 'AppDescription':
        return self.app.description

    @property
    def ip_address(self) -> str:
        ip_address = self._dockerblade.ip_address
        assert ip_address
        return ip_address

    @property
    def messages(self) -> TypeDatabase:
        return self.app.description.types

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
        return CatkinTools(shell=self.shell,
                           directory=directory,
                           files=self.files)

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
        return CatkinMake(shell=self.shell,
                          directory=directory,
                          files=self.files)

    @contextlib.contextmanager
    def ros1(self, port: int = 11311) -> Iterator[ROS1]:
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
        ROS1
            An interface to the launched ROS Master.
        """
        assert port > 1023
        command = f"roscore -p {port}"
        process = self.shell.popen(command)
        try:
            yield ROS1(description=self.app.description,
                       shell=self.shell,
                       files=self.files,
                       ws_host=self._host_workspace,
                       ip_address=self.ip_address,
                       port=port)
        finally:
            process.terminate()
            process.wait(2.0)
            process.kill()

    @property
    def ros2(self) -> ROS2:
        """Provides access to ROS2 inside this application instance."""
        return ROS2.for_app_instance(self)

    def close(self) -> None:
        """Closes this application instance and destroys all resources."""
        self._dockerblade.remove()

        workspace = self._host_workspace
        if os.path.exists(workspace):
            logger.debug(f"destroying app instance directory: {workspace}")
            shutil.rmtree(workspace)
            logger.debug(f"destroyed app instance directory: {workspace}")

    def persist(self,
                repo: Optional[str] = None,
                tag: Optional[str] = None
                ) -> DockerImage:
        """Persists this application instance to a Docker image.

        Parameters
        ----------
        repo: str, optional
            The name of the repository to which the image should belong.
        tag: str, optional
            The tag that should be used for the image.

        Returns
        -------
        DockerImage
            A description of the persisted image.
        """
        return self._dockerblade.persist(repo, tag)

    def __enter__(self) -> 'AppInstance':
        return self

    def __exit__(self,
                 ex_type: Optional[Type[BaseException]],
                 ex_val: Optional[BaseException],
                 ex_tb: Optional[TracebackType]
                 ) -> None:
        self.close()
