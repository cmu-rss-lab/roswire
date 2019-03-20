__all__ = ('ShellProxy',)

from typing import Tuple, Optional, Dict, Any, Iterator
from timeit import default_timer as timer
import os
import shlex
import logging
import threading
import signal

from docker import DockerClient
from docker import APIClient as DockerAPIClient
from docker.models.containers import Container as DockerContainer

from ..util import Stopwatch

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class Popen:
    def __init__(self,
                 args: str,
                 uid: str,
                 shell: 'ShellProxy',
                 api_docker: DockerAPIClient,
                 exec_id: int,
                 stream: Iterator[bytes]
                 ) -> None:
        self.__args = args
        self.__uid = uid
        self.__shell = shell
        self.__api_docker = api_docker
        self.__exec_id = exec_id
        self.__stream = stream
        self.__pid: Optional[int] = None
        self.__returncode: Optional[int] = None

    def _inspect(self) -> Dict[str, Any]:
        return self.__api_docker.exec_inspect(self.__exec_id)

    @property
    def stream(self) -> Iterator[str]:
        for line_bytes in self.__stream:
            yield line_bytes.decode('utf-8')

    @property
    def args(self) -> str:
        return self.__args

    def __uid_to_pid(self) -> Optional[int]:
        prefix = f'/bin/bash -c echo {self.__uid} > /dev/null && '
        cmd = f'ps -eo pid,cmd | grep "{prefix}"'
        while not self.finished:
            code, output, duration = self.__shell.execute(cmd)
            for line in output.split('\n'):
                pid_str, _, p_cmd = line.strip().partition(' ')
                if p_cmd.startswith(prefix):
                    return int(pid_str)
        return None

    @property
    def pid(self) -> Optional[int]:
        if not self.__pid and not self.finished:
            self.__pid = self.__uid_to_pid()
        return self.__pid

    @property
    def finished(self) -> bool:
        return self.returncode is not None

    @property
    def returncode(self) -> Optional[int]:
        if self.__returncode is None:
            self.__returncode = self._inspect()['ExitCode']
        return self.__returncode

    def send_signal(self, sig: int) -> None:
        pid = self.pid
        if pid:
            self.__shell.send_signal(pid, sig)

    def kill(self) -> None:
        self.send_signal(signal.SIGKILL)

    def terminate(self) -> None:
        self.send_signal(signal.SIGTERM)


class ShellProxy:
    """
    Provides shell access for a given BugZoo container.
    """
    def __init__(self,
                 api_docker: DockerAPIClient,
                 container_docker: DockerContainer
                 ) -> None:
        self.__api_docker = api_docker
        self.__container_docker = container_docker

    def send_signal(self, pid: int, sig: int) -> None:
        self.execute(f'kill -{sig} {pid}', user='root')

    def __generate_popen_uid(self, command: str) -> str:
        id_thread = threading.get_ident()
        h = abs(hash((id_thread, timer(), command)))
        return str(h)

    def instrument(self,
                   command: str,
                   time_limit: Optional[int] = None,
                   kill_after: int = 1,
                   identifier: Optional[str] = None
                   ) -> str:
        logger.debug("instrumenting command: %s", command)
        q = shlex.quote
        command = f'source /.environment && {command}'
        if identifier:
            command = f'echo {q(identifier)} > /dev/null && {command}'
        command = f'/bin/bash -c {q(command)}'
        if time_limit:
            command = (f'timeout --kill-after={kill_after} '
                       f'--signal=SIGTERM {time_limit} {command}')
        logger.debug("instrumented command: %s", command)
        return command

    def popen(self,
              command: str,
              *,
              stdout: bool = True,
              stderr: bool = True,
              user: Optional[str] = None,
              context: Optional[str] = None,
              time_limit: Optional[int] = None,
              kill_after: int = 1
              ) -> Popen:
        uid_popen = self.__generate_popen_uid(command)
        id_container = self.__container_docker.id
        api_docker = self.__api_docker
        command_orig = command
        command = self.instrument(command, time_limit, kill_after,
                                  identifier=uid_popen)
        exec_resp = api_docker.exec_create(id_container, command,
                                           tty=True,
                                           stdout=stdout,
                                           stderr=stderr)
        exec_id = exec_resp['Id']
        stream = api_docker.exec_start(exec_id, stream=True)
        return Popen(command_orig,
                     uid_popen,
                     self,
                     api_docker,
                     exec_id,
                     stream)

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

    def non_blocking_execute(self,
                             *args,
                             **kwargs
                             ) -> None:
        thread = threading.Thread(target=ShellProxy.execute,
                                  args=(self, *args),
                                  kwargs=kwargs)
        thread.start()
