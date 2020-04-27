# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchManager',)

from typing import List, Mapping, Optional, Sequence, Union
import os
import shlex

from loguru import logger
import attr
import dockerblade

from .config import LaunchConfig
from .reader import LaunchFileReader
from ... import exceptions as exc


@attr.s(eq=False)
class ROSLaunchManager:
    _shell: dockerblade.shell.Shell = attr.ib(repr=False)
    _files: dockerblade.files.FileSystem = attr.ib(repr=False)

    def __call__(self, *args, **kwargs) -> None:
        """Provides an alias for :code:`launch`."""
        return self.launch(*args, **kwargs)

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
        filename = self.locate(filename, package=package)
        reader = LaunchFileReader(shell=self._shell, files=self._files)
        return reader.read(filename, argv)

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

        filename_original = filename
        logger.debug(f'determing location of launch file [{filename}]'
                     f' in package [{package}]')
        command = f'rospack find {shlex.quote(package)}'
        try:
            path = self._shell.check_output(command, stderr=False)
        except dockerblade.CalledProcessError as err:
            raise exc.PackageNotFound(package) from err
        path = os.path.join(path, 'launch', filename)
        if not self._files.isfile(path):
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
        filename = self.locate(filename, package=package)

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
