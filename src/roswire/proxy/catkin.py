# -*- coding: utf-8 -*-
__all__ = ('CatkinProxy', 'CatkinToolsProxy', 'CatkinMakeProxy')

from typing import Optional, List
import abc
import shlex
import logging

from .shell import ShellProxy
from ..exceptions import CatkinBuildFailed, CatkinCleanFailed

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class CatkinProxy(abc.ABC):
    def __init__(self, shell: ShellProxy, directory: str) -> None:
        """Constructs a catkin proxy for a given workspace."""
        self._directory = directory
        self._shell = shell

    @property
    def directory(self) -> str:
        """The directory of this catkin workspace."""
        return self._directory

    @abc.abstractmethod
    def clean(self,
              packages: Optional[List[str]] = None,
              orphans: bool = False,
              context: Optional[str] = None
              ) -> None:
        """Cleans all products of the catkin workspace.

        Raises
        ------
        CatkinCleanFailed:
            if the attempt to clean resulted in a non-zero return code.
        """
        ...

    @abc.abstractmethod
    def build(self,
              packages: Optional[List[str]] = None,
              no_deps: bool = False,
              pre_clean: bool = False,
              jobs: Optional[int] = None,
              cmake_args: Optional[List[str]] = None,
              make_args: Optional[List[str]] = None,
              context: Optional[str] = None,
              time_limit: Optional[int] = None
              ) -> None:
        """Attempts to build a catkin workspace.

        Raises
        ------
        CatkinBuildFailed:
            if the attempt to build resulted in a non-zero return code.
        """
        ...


class CatkinToolsProxy(CatkinProxy):
    """Provides an interface to a catkin workspace created via catkin tools."""
    def clean(self,
              packages: Optional[List[str]] = None,
              orphans: bool = False,
              context: Optional[str] = None
              ) -> None:
        shell = self._shell
        command = ['catkin', 'clean', '-y']
        if orphans:
            command += ['--orphans']
        if packages:
            command += [shlex.quote(p) for p in packages]
        if not context:
            context = self.directory

        command_str = ' '.join(command)
        logger.debug("cleaning via: %s", command_str)
        retcode, output, duration_secs = \
            shell.execute(command_str, context=context)
        duration_mins = duration_secs / 60
        logger.debug("clean completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, retcode, output)

        if retcode != 0:
            raise CatkinCleanFailed(retcode, output)

    def build(self,
              packages: Optional[List[str]] = None,
              no_deps: bool = False,
              pre_clean: bool = False,
              jobs: Optional[int] = None,
              cmake_args: Optional[List[str]] = None,
              make_args: Optional[List[str]] = None,
              context: Optional[str] = None,
              time_limit: Optional[int] = None
              ) -> None:
        shell = self._shell
        command = ['catkin', 'build', '--no-status', '--no-notify']
        if packages:
            command += [shlex.quote(p) for p in packages]
        if no_deps:
            command += ['--no-deps']
        if pre_clean:
            command += ['--pre-clean']
        if cmake_args:
            command += [shlex.quote(a) for a in cmake_args]
        if make_args:
            command += [shlex.quote(a) for a in make_args]
        if not context:
            context = self.directory

        command_str = ' '.join(command)
        logger.debug("building via: %s", command_str)
        retcode, output, duration_secs = \
            shell.execute(command_str, context=context, time_limit=time_limit)
        duration_mins = duration_secs / 60
        logger.debug("build completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, retcode, output)

        if retcode != 0:
            raise CatkinBuildFailed(retcode, output)


class CatkinMakeProxy(CatkinProxy):
    """Provides an interface to a catkin workspace created via catkin_make."""
    def clean(self,
              packages: Optional[List[str]] = None,
              orphans: bool = False,
              context: Optional[str] = None
              ) -> None:
        raise NotImplementedError

    def build(self,
              packages: Optional[List[str]] = None,
              no_deps: bool = False,
              pre_clean: bool = False,
              jobs: Optional[int] = None,
              cmake_args: Optional[List[str]] = None,
              make_args: Optional[List[str]] = None,
              context: Optional[str] = None,
              time_limit: Optional[int] = None
              ) -> None:
        raise NotImplementedError
