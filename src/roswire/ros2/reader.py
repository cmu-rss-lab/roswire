# -*- coding: utf-8 -*-
import json
import os
import shlex
import typing
from typing import Any, Dict, Optional, Sequence, Tuple

import attr
import pkg_resources
from loguru import logger

from ..proxy.roslaunch.config import ExecutableType, LaunchConfig, NodeConfig
from ..proxy.roslaunch.context import LaunchContext
from ..proxy.roslaunch.reader import LaunchFileReader

if typing.TYPE_CHECKING:
    from .. import AppInstance


@attr.s(auto_attribs=True)
class ROS2LaunchFileReader(LaunchFileReader):

    _app_instance: 'AppInstance'

    def locate_node_binary(self, package: str, node_type: str) -> str:
        raise NotImplementedError

    def read(self,
             filename: str,
             argv: Optional[Sequence[str]] = None
             ) -> LaunchConfig:
        """
        Produces a summary of the effects of a launch file.

        Parameters
        ----------
        filename: str
            The name of the launch file, or an absolute path to the launch
            file inside the container.
        argv: Sequence[str], optional
            An optional sequence of command-line arguments that should be
            supplied to :code:`roslaunch`.

        Raises
        ------
        LaunchFileNotFound
            If the given launch file could not be found in the package.
        """
        if self._app_instance is None:
            raise ValueError('ROS2 version requires and AppInstance object')
        logger.debug("Copying launch extraction script")
        files = self._app_instance.files
        host_script = pkg_resources.resource_filename(
            'roswire',
            'resources/launch_extractor._py')
        files.copy_from_host(host_script,
                             '/launch_extractor.py')

        config_nodes = self.process_launch_on_app_instance(filename, files)

        lc = LaunchConfig()
        ctx = LaunchContext(namespace='/', filename=filename)
        if argv:
            ctx = ctx.with_argv(argv)

        ctx, cfg = self._load_launch_objects(ctx, lc,
                                             [list(config_nodes)])
        logger.debug(f'launch configuration: {cfg}')
        return lc

    def process_launch_on_app_instance(self,
                                       filename: str,
                                       files: Any) -> Sequence[Dict[str, Any]]:
        assert self._app_instance is not None
        output = shlex.quote(os.path.basename(filename) + '.json')
        cmd = f'python3 /launch_extractor.py --output' \
              f' {output} {shlex.quote(filename)}'
        logger.debug(f"Running the script in the container: {cmd}")
        self._app_instance.shell.check_call(cmd)
        logger.debug(f"Reading {output} on container")
        config_json = files.read(output)
        config_nodes = json.loads(config_json)
        return config_nodes

    def _load_launch_objects(self,
                             ctx: LaunchContext,
                             cfg: LaunchConfig,
                             node_list: Sequence[Sequence[Dict[str, Any]]]
                             ) -> Tuple[LaunchContext, LaunchConfig]:
        for nodes in node_list:
            for node in nodes:
                if node['__TYPE__'] == 'Node':
                    args = ' '.join(node.get('args', []))
                    remappings = tuple(node.get('remappings', []))
                    nc = NodeConfig(
                        name=node['name'],
                        namespace=node['namespace'],
                        package=node['package'],
                        executable_path=node['executable_path'],
                        executable_type=ExecutableType[
                            node['executable_type']
                        ],
                        remappings=remappings,
                        filename=node.get('filename'),
                        output=node.get('output'),
                        required=node.get('required', False),
                        respawn=node.get('respawn', False),
                        respawn_delay=node.get('respawn_delay', 0.0),
                        env_args=tuple(node.get('env_args', [])),
                        cwd=node.get('cwd'),
                        args=args,
                        launch_prefix=node.get('launch_prefix'),
                        # ROS 2 has no type in nodes, derive it from
                        # the fulle executable path
                        typ=os.path.basename(node['executable_path'])
                    )
                    cfg = cfg.with_node(nc)
                elif node['__TYPE__'] == 'ExecuteProcess' \
                        and node['cmd'][0] == 'gazebo':
                    raise NotImplementedError
        return ctx, cfg
