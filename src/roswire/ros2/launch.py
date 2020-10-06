# -*- coding: utf-8 -*-
__all__ = ('ROS2LaunchManager',)

import os
import shlex
import json
import typing
from typing import Collection, List, Mapping, Optional, Sequence, Tuple, Union, Dict

import attr
import yaml
from loguru import logger

from .. import exceptions as exc
from ..proxy.roslaunch.config import LaunchConfig, NodeConfig
from ..proxy.roslaunch.context import LaunchContext
from ..proxy.roslaunch.controller import ROSLaunchController

if typing.TYPE_CHECKING:
    from ..app import AppInstance


@attr.s(eq=False)
class ROS2LaunchManager:
    """Provides access to `ros2 launch
    <design.ros2.org/articles/roslaunch.html>`_ for an
    associated ROS2 system. This interface is used to locate, read,
    and write `launch python files and to launch ROS nodes using those
    files.
    """
    _app_instance: 'AppInstance' = attr.ib()

    @classmethod
    def for_app_instance(cls,
                         app_instance: 'AppInstance'
                         ) -> 'ROS2LaunchManager':
        return ROS2LaunchManager(app_instance=app_instance)

    def _load_launch_objects(self, ctx: LaunchContext,
                             cfg: LaunchConfig,
                             node_list: Sequence[Sequence[Dict]]):
        for nodes in node_list:
            for node in nodes:
                if node['__TYPE__'] == 'Node':
                    remappings =  self._get_remappings_from_json(node.get('remappings'))
                    nc = NodeConfig(
                        name=node['name'],
                        namespace=node['namespace'],
                        package=node['package'],
                        executable_path=node['executable_path'],
                        executable_type=None,
                        remappings=remappings,
                        filename=node.get('filename'),
                        output=node.get('output'),
                        required=self._get_bool(node.get('required'), False),
                        respawn=self._get_bool(node.get('respawn'), False),
                        respawn_delay=self._get_float(node.get('respawn_delay'), 0.0),
                        env_args=node.get('env_args'),
                        cwd=node.get('cwd'),
                        args=node.get('args'),
                        launch_prefix=node.get('launch_prefix')
                    )
                    cfg.with_node(nc)
                elif node['__TYPE__'] == 'ExecuteProcess' and node['cmd'][0] == 'gazebo':
                    nc = NodeConfig(
                        name='gazebo',
                        args=node['cmd'][1:],
                    )
                    cfg.with_node(nc)
        return ctx, cfg

    def _get_bool(self, v: str, _default: bool):
        return bool(str) if str is not None else _default

    def read(self,
             filename: str,
             *,
             package: Optional[str] = None,
             argv: Optional[Sequence[str]] = None
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

        # Copy resources/launch_extractor._py as a python file into the container
        logger.debug("Copying launch extraction script")
        files = self._app_instance.files
        files.copy_from_host('resources/launch_extractor.py',
                             '/launch_extractor.py')

        # Runs the script using app_instance.shell. This will create an arch.json
        logger.debug("Running the script in the container")
        output = shlex.quote(os.path.basename(filename) + '.json')
        cmd = f'python /launch_extractor.py --output {output} {shlex.quote(filename)}'
        # This will write the file to /arch.json
        self._app_instance.shell.popen(cmd, stdout=True, stderr=True)
        logger.debug(f"Reading arch.json on container")
        config_json = files.read(output)
        config_nodes = json.loads(config_json)

        # Convert json to a launch config
        # TODO Implement launch config
        lc = LaunchConfig()
        ctx = LaunchContext(namespace='/', filename=filename)
        if argv:
            ctx = ctx.with_argv(argv)

        ctx, cfg = self._load_launch_objects(ctx, lc, list(config_nodes))
        logger.debug(f'launch configuration: {cfg}')
        return lc

    def write(self,
              config: LaunchConfig,
              *,
              filename: Optional[str] = None
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
        str
            The absolute path to the generated XML launch file.
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
                    logger.debug('determined location of launch file'
                                 f' [{filename}] in package [{package}]: '
                                 f'{path}')
                    return path
        raise exc.LaunchFileNotFound(path=filename)

    def launch(self,
               filename: str,
               *,
               package: Optional[str] = None,
               args: Optional[Mapping[str, Union[int, str]]] = None,
               prefix: Optional[str] = None,
               launch_prefixes: Optional[Mapping[str, str]] = None,
               node_to_remappings: Optional[Mapping[str, Collection[Tuple[str, str]]]] = None
               # noqa
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
            cmd = ['ros2 launch', shlex.quote(package),
                   shlex.quote(filename_without_path)]
        else:
            m = "Not yet implemented when package is None"
            raise NotImplementedError(m)
        launch_args: List[str] = [f'{arg}:={val}' for arg, val in args.items()]
        cmd += launch_args
        if prefix:
            cmd = [prefix] + cmd
        cmd_str = ' '.join(cmd)
        popen = shell.popen(cmd_str, stdout=True, stderr=True)
        return ROSLaunchController(filename=filename,
                                   popen=popen)

    __call__ = launch
