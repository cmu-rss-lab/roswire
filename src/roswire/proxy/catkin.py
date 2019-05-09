# -*- coding: utf-8 -*-
__all__ = ('CatkinProxy',)

from typing import Optional, List
import shlex

from shell import ShellProxy


class CatkinProxy:
    def __init__(self, shell: ShellProxy) -> None:
        self._shell = shell

    def build(self,
              packages: Optional[List[str]] = None,
              no_deps: bool = False,
              pre_clean: bool = False,
              jobs: Optional[int] = None,
              cmake_args: Optional[List[str]] = None,
              make_args: Optional[List[str]] = None
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

        # TODO check exit code -- 127 indicates catkin_tools not installed
