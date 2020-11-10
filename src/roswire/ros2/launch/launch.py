# -*- coding: utf-8 -*-
__all__ = ("ROS2LaunchManager",)

import os
import shlex
import typing
from typing import Collection, List, Mapping, Optional, Sequence, Tuple, Union

import attr
from loguru import logger

from .reader import ROS2LaunchFileReader
from ... import exceptions as exc
from ...common.launch.config import LaunchConfig
from ...common.launch.controller import ROSLaunchController

if typing.TYPE_CHECKING:
    from ... import AppInstance


@attr.s(eq=False)
class ROS2LaunchManager:
    """
    Provides access to `ros2 launch
    <design.ros2.org/articles/roslaunch.html>`_ for an
    associated ROS2 system. This interface is used to locate, read,
    and write `launch python files and to launch ROS nodes using those
    files.
    """

    _app_instance: "AppInstance" = attr.ib()

    @classmethod
    def for_app_instance(
        cls, app_instance: "AppInstance"
    ) -> "ROS2LaunchManager":
        return ROS2LaunchManager(app_instance=app_instance)

    def read(
        self,
        filename: str,
        *,
        package: Optional[str] = None,
        argv: Optional[Sequence[str]] = None,
    ) -> LaunchConfig:
        """Produces a summary of the effects of a launch file.

        Parameters
        ----------
        filename: str
            The name of the launch file, or an absolute path to the launch
            file inside the container.
        package: str, optional
            The name of the package to which the launch file belongs.
        argv: Sequence[str], optional
            An optional sequence of command-line arguments that should be
            supplied to :code:`roslaunch`.

        Raises
        ------
        PackageNotFound
            If the given package could not be found.
        LaunchFileNotFound
            If the given launch file could not be found in the package.
        """
        filename = self.locate(filename, package=package)
        reader = ROS2LaunchFileReader(self._app_instance)
        return reader.read(filename, argv)

    def write(
        self, config: LaunchConfig, *, filename: Optional[str] = None
    ) -> str:
        """Writes a given launch configuration to disk as an XML launch file.

        Parameters
        ----------
        config: LaunchConfig
            A launch configuration.
        filename: str, optional
            The name of the file to which the configuration should be written.
            If no filename is given, a temporary file will be created. It is
            the responsibility of the caller to ensure that the temporary file
            is appropriately destroyed.

        Returns
        -------
        str: The absolute path to the generated XML launch file.
        """
        raise NotImplementedError

    def locate(self, filename: str, *, package: Optional[str] = None) -> str:
        """Locates a given launch file.

        Parameters
        ----------
        filename: str
            The name of the launch file, or an absolute path to the launch
            file inside the container.
        package: str, optional
            Optionally specifies the name of the package to which the launch
            file belongs.

        Returns
        -------
        The absolute path to the launch file, if it exists.

        Raises
        ------
        PackageNotFound
            If the given package could not be found.
        LaunchFileNotFound
            If the given launch file could not be found in the package.
        """
        if not package:
            assert os.path.isabs(filename)
            return filename
        else:
            app_description = self._app_instance.app.description
            package_description = app_description.packages[package]
            package_location = package_description.path
            paths = self._app_instance.files.find(package_location, filename)
            for path in paths:
                if package in path:
                    logger.debug(
                        "determined location of launch file"
                        f" [{filename}] in package [{package}]: "
                        f"{path}"
                    )
                    return path
        raise exc.LaunchFileNotFound(path=filename)

    def launch(
        self,
        filename: str,
        *,
        package: Optional[str] = None,
        args: Optional[Mapping[str, Union[int, str]]] = None,
        prefix: Optional[str] = None,
        launch_prefixes: Optional[Mapping[str, str]] = None,
        node_to_remappings: Optional[
            Mapping[str, Collection[Tuple[str, str]]]
        ] = None,
    ) -> ROSLaunchController:
        """Provides an interface to the roslaunch command.

        Parameters
        ----------
        filename: str
            The name of the launch file, or an absolute path to the launch
            file inside the container.
        package: str, optional
            The name of the package to which the launch file belongs.
        args: Dict[str, Union[int, str]], optional
            Keyword arguments that should be supplied to roslaunch.
        prefix: str, optional
            An optional prefix to add before the roslaunch command.
        launch_prefixes: Mapping[str, str], optional
            An optional mapping from nodes, given by their names, to their
            individual launch prefix.
        node_to_remappings: Mapping[str, Collection[Tuple[str, str]]], optional
            A collection of name remappings for each node, represented as a
            mapping from node names to a collection of remappings for that
            node, where each remapping is a tuple of the
            form :code:`(to, from)`.

        Returns
        -------
        ROSLaunchController
            An interface for inspecting and managing the launch process.

        Raises
        ------
        PackageNotFound
            If the given package could not be found.
        LaunchFileNotFound
            If the given launch file could not be found in the package.
        """
        shell = self._app_instance.shell
        if not args:
            args = {}
        if not launch_prefixes:
            launch_prefixes = {}

        if node_to_remappings or launch_prefixes:
            m = "Requires self.read: not yet implemented"
            raise NotImplementedError(m)
        if package:
            filename_without_path = os.path.basename(filename)
            cmd = [
                "ros2 launch",
                shlex.quote(package),
                shlex.quote(filename_without_path),
            ]
        else:
            m = "Not yet implemented when package is None"
            raise NotImplementedError(m)
        launch_args: List[str] = [f"{arg}:={val}" for arg, val in args.items()]
        cmd += launch_args
        if prefix:
            cmd = [prefix] + cmd
        cmd_str = " ".join(cmd)
        popen = shell.popen(cmd_str, stdout=True, stderr=True)
        return ROSLaunchController(filename=filename, popen=popen)

    __call__ = launch
