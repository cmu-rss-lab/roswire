__all__ = ['Container']

from typing import Iterator
from uuid import UUID
import contextlib

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer

from .exceptions import RozzyException
from .proxy import ShellProxy, ROSProxy


class Container(object):
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer,
                 uuid: UUID
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo
        self.__uuid = uuid
        self.__shell = ShellProxy(daemon_bugzoo, container_bugzoo)

    @property
    def uuid(self) -> UUID:
        return self.__uuid

    @property
    def ip_address(self) -> str:
        bz = self.__daemon_bugzoo
        ip = bz.containers.ip_address(self.__container_bugzoo)
        if not ip:
            m = "no IP address for container: {}"
            m = m.format(self.uuid.hex)
            raise RozzyException(m)
        return str(ip)

    @property
    def shell(self) -> ShellProxy:
        return self.__shell

    @contextlib.contextmanager
    def roscore(self) -> Iterator[ROSProxy]:
        # http://wiki.ros.org/ROS/NetworkSetup
        # http://ros-users.122217.n3.nabble.com/Current-best-practice-for-multiple-interfaces-td4019355.html
        # https://answers.ros.org/question/297713/localhost-vs-0000/
        self.shell.non_blocking_execute("roscore -p 11311")
        try:
            yield ROSProxy(shell=self.shell,
                           ip_address=self.ip_address,
                           port=11311)
        finally:
            self.shell.execute("pkill roscore")
