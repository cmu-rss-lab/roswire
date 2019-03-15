__all__ = ('ShellProxy',)

from typing import Tuple, Optional
import shlex
import asyncio
import logging

from docker.models.containers import Container as DockerContainer

from ..util import Stopwatch

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class ShellProxy:
    """
    Provides shell access for a given BugZoo container.
    """
    def __init__(self, container_docker: DockerContainer) -> None:
        self.__container_docker = container_docker

    def instrument(self,
                   command: str,
                   time_limit: Optional[int] = None,
                   kill_after: int = 1
                   ) -> str:
        logger.debug("instrumenting command: %s", command)
        q = shlex.quote
        command = f'source /.environment && {command}'
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
        command = self.instrument(command, time_limit, kill_after)

        timer = Stopwatch()
        timer.start()
        # https://docker-py.readthedocs.io/en/stable/containers.html
        retcode, output = \
            dockerc.exec_run(command, stdout=stdout, stderr=stderr, workdir=context)  # noqa
        timer.stop()
        duration = timer.duration
        output = output.decode('utf-8').rstrip('\n')
        logger.debug("executed command [%s] (retcode: %d; time: %.3f s)\n%s",
                     command, retcode, timer.duration, output)
        return retcode, output, timer.duration

    async def non_blocking_execute(self,
                                   *args,
                                   **kwargs
                                   ) -> Tuple[int, str, float]:
        return self.execute(*args, **kwargs)
