# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchManager',)

from typing import List, Mapping, Optional, Union
import os
import shlex

from loguru import logger
import attr
import dockerblade


@attr.s(eq=False)
class ROSLaunchManager:
    _shell: dockerblade.shell.Shell = attr.ib(repr=False)
    _files: dockerblade.files.FileSystem = attr.ib(repr=False)

    def launch(self,
               filename: str,
               *,
               package: Optional[str] = None,
               args: Optional[Mapping[str, Union[int, str]]] = None,
               prefix: Optional[str] = None,
               launch_prefixes: Optional[Mapping[str, str]] = None
               ) -> None:
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
        """
        shell = self._shell
        if not args:
            args = {}
        if not launch_prefixes:
            launch_prefixes = {}
        launch_args: List[str] = [f'{arg}:={val}' for arg, val in args.items()]

        if launch_prefixes:
            m = "individual launch prefixes are not yet implemented"
            raise NotImplementedError(m)

        # determine the absolute path of the launch file
        if package:
            filename_original = filename
            logger.debug(f'determing location of launch file [{filename}]'
                         f' in package [{package}]')
            package_escaped = shlex.quote(package)
            find_package_command = f'rospack find {package_escaped}'
            package_path = shell.check_output(find_package_command,
                                              stderr=False)
            filename = os.path.join(package_path, 'launch', filename)
            logger.debug('determined location of launch file'
                         f' [{filename_original}] in package [{package}]: '
                         f'{filename}')

        cmd = ['roslaunch', shlex.quote(filename)]
        cmd += launch_args
        if prefix:
            cmd = [prefix] + cmd
        cmd_str = ' '.join(cmd)
        shell.popen(cmd_str, stdout=False, stderr=False)


