# -*- coding: utf-8 -*-
__all__ = ('Node',)

from typing import Optional
import xmlrpc.client

import dockerblade
import psutil

from ..exceptions import ROSWireException


class Node:
    """Provides access to a ROS node.

    Attributes
    ----------
    api: xmlrpc.client.ServerProxy
        An XML-RPC API client for this node.
    name: str
        The fully qualified name of this node.
    url: str
        URL used to access this node from the host network.
    pid: int
        The container PID of the main process for this node.
    pid_host: int
        The host PID of the main process for this node.
    """
    def __init__(self,
                 name: str,
                 url_host_network: str,
                 shell: dockerblade.Shell
                 ) -> None:
        self.__name = name
        self.__url = url_host_network
        self.__shell = shell
        self.__pid_host: Optional[int] = None

    @property
    def api(self) -> xmlrpc.client.ServerProxy:
        return xmlrpc.client.ServerProxy(self.url)

    @property
    def name(self) -> str:
        return self.__name

    @property
    def url(self) -> str:
        return self.__url

    @property
    def pid(self) -> int:
        code, status, pid = self.api.getPid('/.roswire')  # type: ignore
        if code != 1:
            m = f"failed to obtain PID [{self.name}]: {status} (code: {code})"
            raise ROSWireException(m)
        assert isinstance(pid, int)
        assert pid > 0
        return pid

    @property
    def pid_host(self) -> int:
        if self.__pid_host is None:
            self.__pid_host = self.__shell._local_to_host_pid(self.pid)
            assert self.__pid_host is not None
        return self.__pid_host

    def is_alive(self) -> bool:
        """Determines whether this node is alive."""
        # TODO check start time to ensure this is the same process!
        try:
            return psutil.pid_exists(self.pid_host)
        except ROSWireException:
            return False

    def shutdown(self) -> None:
        """Instructs this node to shutdown."""
        self.__shell.run(f'rosnode kill {self.name}')
