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
    from ..app import AppInstance


@attr.s(eq=False)
class ROS2LaunchFileReader(LaunchFileReader):

    app_instance: 'AppInstance'

    def locate_node_binary(self, package: str, node_type: str) -> str:
        pass

    def read(self,
             filename: str,
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

        # Copy resources/launch_extractor._py as a python
        # file into the container
        logger.debug("Copying launch extraction script")
        files = self.app_instance.files
        host_script = pkg_resources. \
            resource_filename('roswire',
                              'resources/launch_extractor._py')
        logger.debug(f'Resource is {host_script}')
        files.copy_from_host(host_script,
                             '/launch_extractor.py')

        # Runs the script using app_instance.shell.
        output = shlex.quote(os.path.basename(filename) + '.json')
        cmd = f'python3 /launch_extractor.py --output' \
              f' {output} {shlex.quote(filename)}'
        logger.debug(f"Running the script in the container: {cmd}")
        result = self.app_instance.shell.check_call(cmd)
        logger.debug(f"Call resulted in {result}")
        logger.debug(f"Reading {output} on container")
        config_json = files.read(output)
        config_nodes = json.loads(config_json)
        # Convert json to a launch config
        # TODO Implement launch config
        lc = LaunchConfig()
        ctx = LaunchContext(namespace='/', filename=filename)
        if argv:
            ctx = ctx.with_argv(argv)

        ctx, cfg = self._load_launch_objects(ctx, lc,
                                             [list(config_nodes)])
        logger.debug(f'launch configuration: {cfg}')
        return lc

    def _load_launch_objects(self, ctx: LaunchContext,
                             cfg: LaunchConfig,
                             node_list: Sequence[Sequence[Dict]]
                             ) -> Tuple[LaunchContext, LaunchConfig]:
        for nodes in node_list:
            for node in nodes:
                if node['__TYPE__'] == 'Node':
                    print(f'processing: {node}')
                    # remappings = self.\
                    #     _get_remappings_from_json(node.get('remappings'))
                    args: Optional[Any] = node.get('args', [])
                    if isinstance(args, list):
                        args = ' '.join(args)
                    print(args)
                    remappings = node.get('remappings', [])
                    if remappings is not None:
                        remappings = tuple(remappings)
                    nc = NodeConfig(
                        name=node['name'],
                        namespace=node['namespace'],
                        package=node['package'],
                        executable_path=node['executable_path'],
                        executable_type=ExecutableType[
                            node['executable_type']
                        ],
                        remappings=remappings, # noqa
                        filename=node.get('filename'),
                        output=node.get('output'),
                        required=self._get_bool(node.get('required'), False),
                        respawn=self._get_bool(node.get('respawn'), False),
                        respawn_delay=self._get_float(
                            node.get('respawn_delay'), 0.0),
                        env_args=tuple(node.get('env_args', [])), # noqa
                        cwd=node.get('cwd'),
                        args=args,
                        launch_prefix=node.get('launch_prefix'),
                        typ=''
                    )
                    cfg.with_node(nc)
                elif node['__TYPE__'] == 'ExecuteProcess' \
                        and node['cmd'][0] == 'gazebo':
                    nc = NodeConfig(
                        name='gazebo',
                        namespace='',
                        typ='',
                        package='',
                        executable_path=node['cmd'][0],
                        executable_type=ExecutableType.LIKELY_CPP,
                        args=' '.join(node['cmd'][1:]),
                    )
                    cfg.with_node(nc)
        return ctx, cfg

    def _get_bool(self, v: Optional[Any], _default: bool) -> bool:
        return bool(v) if v is not None else _default

    def _get_float(self, v: Optional[Any], _default: float) -> float:
        return float(v) if v is not None else _default
