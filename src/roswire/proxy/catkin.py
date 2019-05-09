# -*- coding: utf-8 -*-
__all__ = ('CatkinProxy',)

from typing import Optional, List
import shlex
import logging

from .shell import ShellProxy
from ..exceptions import CatkinBuildFailed

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class CatkinProxy:
    def __init__(self, shell: ShellProxy) -> None:
        self._shell = shell

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

        command_str = ' '.join(command)
        logger.debug("building via: %s", command_str)
        retcode, output, duration_secs = \
            shell.execute(command_str, context=context, time_limit=time_limit)
        duration_mins = duration_secs / 60
        logger.debug("build completed after %.2f minutes [retcode: %d]:\n%s",
                     duration_mins, retcode, output)

        # TODO check exit code -- 127 indicates catkin_tools not installed

        if retcode != 0:
            raise CatkinBuildFailed(retcode, output)
