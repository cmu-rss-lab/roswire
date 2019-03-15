__all__ = ['ShellProxy']

from typing import Tuple, Optional
import shlex

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer
from bugzoo.cmd import PendingExecResponse


class ShellProxy:
    """
    Provides shell access for a given BugZoo container.
    """
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo

    def instrument(self,
                   command: str,
                   context: str,  # TODO may not be needed
                   time_limit: Optional[int] = None,
                   kill_after: int = 1
                   ) -> str:
        q = shlex.quote
        command = 'source /.environment && cd {q(context)} && {command}'
        command = '/bin/bash -c {q(command)}'
        if time_limit:
            command = (f'timeout --kill-after={kill_after} '
                       f'--signal=SIGTERM {time_limit} {command}')
        return command

    def execute(self,
                command: str,
                *,
                stdout: bool = True,
                stderr: bool = True,
                user: Optional[str] = None,
                context: Optional[str] = None,
                time_limit: Optional[int] = None,
                kill_after: int = 1
                ) -> Tuple[int, str, float]:
        command = self.instrument(command, context, time_limit, kill_after)

        # TODO use timer
        # https://docker-py.readthedocs.io/en/stable/containers.html
        retcode, output = dockerc.exec_run(cmd,
                         stdout=stdout,
                         stderr=stderr)


        return r.code, r.output, r.duration

    def non_blocking_execute(self,
                             command: str,
                             **kwargs
                             ) -> PendingExecResponse:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo, command, block=False,
                        **kwargs)
        return r
