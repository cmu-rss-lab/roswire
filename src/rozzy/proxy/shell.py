__all__ = ['ShellProxy']

from typing import Tuple, Optional
import shlex
import logging

from docker.models.containers import Container as DockerContainer
from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer
from bugzoo.cmd import PendingExecResponse

from ..util import Stopwatch

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class ShellProxy:
    """
    Provides shell access for a given BugZoo container.
    """
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 container_docker: DockerContainer
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo
        self.__container_docker = container_docker

    def instrument(self,
                   command: str,
                   context: str,  # TODO may not be needed
                   time_limit: Optional[int] = None,
                   kill_after: int = 1
                   ) -> str:
        logger.debug("instrumenting command: %s", command)
        q = shlex.quote
        command = f'source /.environment && cd {q(context)} && {command}'
        command = f'/bin/bash -c {q(command)}'
        if time_limit:
            command = (f'timeout --kill-after={kill_after} '
                       f'--signal=SIGTERM {time_limit} {command}')
        logger.debug("instrumented command: %s", command)
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
        logger.debug("executing command: %s", command)
        dockerc = self.__container_docker
        if not context:
            context = ''
        command = self.instrument(command, context, time_limit, kill_after)

        timer = Stopwatch()
        timer.start()
        # https://docker-py.readthedocs.io/en/stable/containers.html
        retcode, output = \
            dockerc.exec_run(command, stdout=stdout, stderr=stderr)
        timer.stop()
        logger.debug("executed command [%s] (retcode: %d; time: %.3f s)\n%s",
                     command, retcode, timer.duration, output)
        return retcode, output, timer.duration

    def non_blocking_execute(self,
                             command: str,
                             **kwargs
                             ) -> PendingExecResponse:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo, command, block=False,
                        **kwargs)
        return r
