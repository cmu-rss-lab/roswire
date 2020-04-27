# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchManager',)

from typing import List, Mapping, Optional, Union
import os
import shlex

from loguru import logger
import attr
import dockerblade

from ... import exceptions as exc


@attr.s(eq=False)
class ROSLaunchManager:
    _shell: dockerblade.shell.Shell = attr.ib(repr=False)
    _files: dockerblade.files.FileSystem = attr.ib(repr=False)

    def locate(self,
               package: str,
               filename: str,
               *,
               check_exists: bool = True
               ) -> str:
        """Finds the location of a launch file within a given package.

        Parameters
        ----------
        package: str
            The name of the package.
        filename: str
            The name of the launch file.
        check_exists: bool
            If :code:`True`, an additional check will ensure that the given
            launch file can be found in the launch directory of the package.
            If :code:`False`, an exception will only be raised if the given
            package cannot be found.

        Returns
        -------
        The absolute path to the launch file, if it exists.

        Raises
        ------
        PackageNotFound
            If the given package could not be found.
        LaunchFileNotFound
            If the given launch file could not be found in the package.
            Only raised if :code:`check_exists` is :code:`True`.
        """
        filename_original = filename
        logger.debug(f'determing location of launch file [{filename}]'
                     f' in package [{package}]')
        command = f'rospack find {shlex.quote(package)}'
        try:
            path = self._shell.check_output(command, stderr=False)
        except dockerblade.CalledProcessError as err:
            raise exc.PackageNotFound(package) from err
        path = os.path.join(path, 'launch', filename)
        if check_exists and not self._files.isfile(path):
            raise exc.LaunchFileNotFound(path=path)
        logger.debug('determined location of launch file'
                     f' [{filename_original}] in package [{package}]: '
                     f'{filename}')
        return filename

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

        Raises
        ------
        PackageNotFound
            If the given package could not be found.
        LaunchFileNotFound
            If the given launch file could not be found in the package.
        """
        shell = self._shell
        if not args:
            args = {}
        if not launch_prefixes:
            launch_prefixes = {}
        if package:
            filename = self.locate(filename, package, check_exists=False)

        if not self._files.isfile(filename):
            raise exc.LaunchFileNotFound(filename)

        if launch_prefixes:
            m = "individual launch prefixes are not yet implemented"
            raise NotImplementedError(m)

        cmd = ['roslaunch', shlex.quote(filename)]
        launch_args: List[str] = [f'{arg}:={val}' for arg, val in args.items()]
        cmd += launch_args
        if prefix:
            cmd = [prefix] + cmd
        cmd_str = ' '.join(cmd)
        shell.popen(cmd_str, stdout=False, stderr=False)
