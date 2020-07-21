# -*- coding: utf-8 -*-
"""
This module provides a unified interface for interacting with the catkin
and catkin_make build systems.
"""
__all__ = ('CatkinInterface', 'CatkinTools', 'CatkinMake')

from typing import Optional, List
import abc
import shlex

from loguru import logger
import attr
import dockerblade

from ..exceptions import CatkinBuildFailed, CatkinCleanFailed


class CatkinInterface(abc.ABC):
    """Provides an interface to a catkin-based workspace."""
    @property
    @abc.abstractmethod
    def directory(self) -> str:
        """The directory of this catkin workspace."""
        ...

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


@attr.s(frozen=True, slots=True, auto_attribs=True)
class CatkinTools(CatkinInterface):
    """Provides an interface to a catkin workspace created via catkin tools."""
    directory: str
    _shell: dockerblade.shell.Shell

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
        logger.debug(f"cleaning via: {command_str}")
        result = shell.run(command_str, cwd=context, text=True)
        duration_mins = result.duration / 60
        logger.debug("clean completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, result.returncode, result.output)

        if result.returncode != 0:
            assert isinstance(result.output, str)
            raise CatkinCleanFailed(result.returncode, result.output)

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
        command = ['catkin', 'build', '--no-status', '--no-notify']
        if packages:
            command += [shlex.quote(p) for p in packages]
        if no_deps:
            command += ['--no-deps']
        if pre_clean:
            command += ['--pre-clean']
        if cmake_args:
            command += cmake_args
        if make_args:
            command += make_args
        if not context:
            context = self.directory

        command_str = ' '.join(command)
        logger.debug(f"building via: {command_str}")
        result = self._shell.run(command_str,
                                 cwd=context,
                                 time_limit=time_limit,
                                 text=True)
        duration_mins = result.duration / 60
        logger.debug("build completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, result.returncode, result.output)

        if result.returncode != 0:
            assert isinstance(result.output, str)
            raise CatkinBuildFailed(result.returncode, result.output)


@attr.s(frozen=True, slots=True, auto_attribs=True)
class CatkinMake(CatkinInterface):
    """Provides an interface to a catkin workspace created via catkin_make."""
    directory: str
    _shell: dockerblade.shell.Shell

    def clean(self,
              packages: Optional[List[str]] = None,
              orphans: bool = False,
              context: Optional[str] = None
              ) -> None:
        shell = self._shell
        command = ['catkin_make', 'clean']
        if orphans:
            raise NotImplementedError
        if packages:
            command += ['--pkg']
            command += [shlex.quote(p) for p in packages]
        if not context:
            context = self.directory

        command_str = ' '.join(command)
        logger.debug(f"cleaning via: {command_str}")
        result = shell.run(command_str, cwd=context, text=True)
        duration_mins = result.duration / 60
        logger.debug("clean completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, result.returncode, result.output)

        if result.returncode != 0:
            assert isinstance(result.output, str)
            raise CatkinCleanFailed(result.returncode, result.output)


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
        command = ['catkin_make']
        if packages:
            command += ['--pkg']
            command += [shlex.quote(p) for p in packages]
        if no_deps:
            raise NotImplementedError
        if pre_clean:
            raise NotImplementedError
        if cmake_args:
            command += cmake_args
        if make_args:
            command += ['--make-args']
            command += make_args
        if not context:
            context = self.directory

        command_str = ' '.join(command)
        logger.debug(f"building via: {command_str}")
        result = self._shell.run(command_str,
                                 cwd=context,
                                 time_limit=time_limit,
                                 text=True)
        duration_mins = result.duration / 60
        logger.debug("build completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, result.returncode, result.output)

        if result.returncode != 0:
            assert isinstance(result.output, str)
            raise CatkinBuildFailed(result.returncode, result.output)

