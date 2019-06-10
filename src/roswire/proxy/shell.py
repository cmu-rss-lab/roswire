__all__ = ('ShellProxy',)

from typing import Tuple, Optional, Dict, Any, Iterator, List
from timeit import default_timer as timer
from subprocess import TimeoutExpired
import os
import shlex
import logging
import threading
import time
import signal

import psutil
from docker import DockerClient
from docker import APIClient as DockerAPIClient
from docker.models.containers import Container as DockerContainer

from ..util import Stopwatch
from .. import exceptions

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
        """True if the process has exited; False if not."""
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
        """Kills the process via a SIGKILL signal."""
        self.send_signal(signal.SIGKILL)

    def terminate(self) -> None:
        """Terminates the process via a SIGTERM signal."""
        self.send_signal(signal.SIGTERM)

    def poll(self) -> Optional[int]:
        """Checks if the process has terminated and returns its returncode."""
        return self.returncode

    def wait(self, time_limit: Optional[float] = None) -> int:
        """Blocks until the process has terminated.

        Parameters
        ----------
        time_limit: Optional[float] = None
            An optional time limit.

        Raises
        ------
        subprocess.TimeoutExpired:
            if the process does not terminate within the specified timeout.
        """
        stopwatch = Stopwatch()
        stopwatch.start()
        while not self.finished:
            if time_limit and stopwatch.duration > time_limit:
                raise TimeoutExpired(self.args, time_limit)
            time.sleep(0.05)
        assert self.returncode is not None
        return self.returncode


class ShellProxy:
    """Provides shell access for a given Docker container."""
    def __init__(self,
                 api_docker: DockerAPIClient,
                 container_docker: DockerContainer,
                 container_pid: int
                 ) -> None:
        self.__api_docker = api_docker
        self.__container_docker = container_docker
        self.__container_pid = container_pid

    def exec_id_to_host_pid(self, exec_id: str) -> int:
        """Returns the host PID for a given exec command."""
        return self.__api_docker.exec_inspect(exec_id)['Pid']

    def local_to_host_pid(self, pid_local: int) -> Optional[int]:
        """Finds the host PID for a process inside this shell."""
        ctr_pids = [self.__container_pid]
        info = self.__api_docker.inspect_container(self.__container_docker.id)
        ctr_pids += [self.exec_id_to_host_pid(i) for i in info['ExecIDs']]

        # obtain a list of all processes inside this container
        ctr_procs: List[psutil.Process] = []
        for pid in ctr_pids:
            proc = psutil.Process(pid)
            ctr_procs.append(proc)
            ctr_procs += proc.children(recursive=True)

        # read /proc/PID/status to find the namespace mapping
        for proc in ctr_procs:
            fn_proc = f'/proc/{proc.pid}/status'
            with open(fn_proc, 'r') as fh_proc:
                lines = filter(lambda l: l.startswith('NSpid'),
                               fh_proc.readlines())
                for line in lines:
                    proc_host_pid, proc_local_pid = \
                        [int(p) for p in line.strip().split('\t')[1:3]]
                    if proc_local_pid == pid_local:
                        return proc_host_pid

        return None

    def send_signal(self, pid: int, sig: int) -> None:
        self.execute(f'kill -{sig} {pid}', user='root')

    def __generate_popen_uid(self, command: str) -> str:
        id_thread = threading.get_ident()
        h = abs(hash((id_thread, timer(), command)))
        return str(h)

    def environ(self, var: str) -> str:
        """Reads the value of a given environment variable inside this shell.

        Raises
        ------
        EnvNotFoundError
            if no environment variable exists with the given name.
        """
        cmd = f'test -v {var} && echo "${{{var}}}"'
        retcode, val, duration = self.execute(cmd)
        if retcode != 0:
            raise exceptions.EnvNotFoundError(var)
        return val

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
