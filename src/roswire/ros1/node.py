# -*- coding: utf-8 -*-
__all__ = ("ROS1Node",)

import xmlrpc.client
from typing import Optional

import dockerblade
import psutil

from ..common import Node
from ..exceptions import ROSWireException


class ROS1Node(Node):
    """Provides access to a ROS node.

    Attributes
    ----------
    name: str
        The fully qualified name of this node.
    _url: str
        The URL used to access this node from the host network.
    pid_host: int
        The host PID of the main process for this node.
    """

    def __init__(
        self, name: str, url_host_network: str, shell: dockerblade.Shell
    ) -> None:
        self.__name = name
        self._url = url_host_network
        self.__shell = shell
        self.__pid_host: Optional[int] = None

    @property
    def name(self) -> str:
        return self.__name

    @property
    def _api(self) -> xmlrpc.client.ServerProxy:
        """An XML-RPC API client for interacting with this node."""
        return xmlrpc.client.ServerProxy(self._url)

    @property
    def _pid(self) -> int:
        """The container PID of the main process for this node."""
        code, status, pid = self.api.getPid("/.roswire")  # type: ignore
        if code != 1:
            m = f"failed to obtain PID [{self.name}]: {status} (code: {code})"
            raise ROSWireException(m)
        assert isinstance(pid, int)
        assert pid > 0
        return pid

    @property
    def _pid_host(self) -> int:
        """The host PID of the main process for this node."""
        if self.__pid_host is None:
            self.__pid_host = self.__shell._local_to_host_pid(self._pid)
            assert self.__pid_host is not None
        return self.__pid_host

    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        # TODO check start time to ensure this is the same process!
        try:
            return psutil.pid_exists(self._pid_host)
        except ROSWireException:
            return False

    def shutdown(self) -> None:
        """Instructs this node to shutdown."""
        self.__shell.run(f"rosnode kill {self.name}")
