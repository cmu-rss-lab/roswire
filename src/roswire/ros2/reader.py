import json
import os
import shlex
from typing import Optional, Sequence, Dict

import attr
from loguru import logger

from src.roswire.proxy.roslaunch.config import LaunchConfig, NodeConfig
from src.roswire.proxy.roslaunch.context import LaunchContext
from src.roswire.proxy.roslaunch.reader import LaunchFileReader


@attr.s(eq=False)
class ROS2LaunchFileReader(LaunchFileReader):

    def locate_node_binary(self, package: str, node_type: str) -> str:
        pass

    def read(self, filename: str, argv: Optional[Sequence[str]] = None) -> LaunchConfig:
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

    def _load_launch_objects(self, ctx: LaunchContext,
                             cfg: LaunchConfig,
                             node_list: Sequence[Sequence[Dict]]):
        for nodes in node_list:
            for node in nodes:
                if node['__TYPE__'] == 'Node':
                    remappings = self._get_remappings_from_json(node.get('remappings'))
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