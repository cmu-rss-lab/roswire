__all__ = ('NodeManagerProxy', 'NodeProxy')

from typing import Iterator, Set, Mapping, Optional
from urllib.parse import urlparse
import xmlrpc.client
import logging

import psutil

from .shell import ShellProxy
from ..exceptions import ROSWireException, NodeNotFoundError

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class NodeProxy:
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
                 shell: ShellProxy
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
        code, status, pid = self.api.getPid('/.roswire')
        if code != 1:
            m = f"failed to obtain PID [{self.name}]: {status} (code: {code})"
            raise ROSWireException(m)
        assert isinstance(pid, int)
        assert pid > 0
        return pid

    @property
    def pid_host(self) -> int:
        if self.__pid_host is None:
            self.__pid_host = self.__shell.local_to_host_pid(self.pid)
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
        self.__shell.execute(f'rosnode kill {self.name}')


class NodeManagerProxy(Mapping[str, NodeProxy]):
    def __init__(self,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy,
                 shell: ShellProxy
                 ) -> None:
        self.__host_ip_master: str = host_ip_master
        self.__api: xmlrpc.client.ServerProxy = api
        self.__shell: ShellProxy = shell

    @property
    def api(self) -> xmlrpc.client.ServerProxy:
        return self.__api

    def __get_node_names(self) -> Set[str]:
        """
        Fetches a list of the names of all active nodes.
        """
        names: Set[str] = set()
        code, status, state = self.api.getSystemState('/.roswire')
        for s in state:
            for t, l in s:
                names.update(n for n in l)
        return names

    def __len__(self) -> int:
        """
        Returns a count of the number of active nodes.
        """
        return len(self.__get_node_names())

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of all active nodes.
        """
        yield from self.__get_node_names()

    def __getitem__(self, name: str) -> NodeProxy:
        """Attempts to fetch a given node.

        Parameters
        ----------
        name: str
            The name of the node.

        Returns
        -------
        NodeProxy
            An interface to the given node.

        Raises
        ------
        NodeNotFoundError
            If there is no node with the given name.
        """
        code, status, uri_container = self.api.lookupNode('/.roswire', name)
        if code == -1:
            raise NodeNotFoundError(name)
        if code != 1:
            m = f"unexpected error when attempting to find node [{name}]: {status} (code: {code})"   # noqa: pycodestyle
            raise ROSWireException(m)

        # convert URI to host network
        port = urlparse(uri_container).port
        uri_host = f"http://{self.__host_ip_master}:{port}"
        return NodeProxy(name, uri_host, self.__shell)

    def __delitem__(self, name: str) -> None:
        """
        Shutdown and deregister a given node.

        Raises:
            NodeNotFoundError: no node found with given name.
        """
        try:
            node = self[name]
        except NodeNotFoundError:
            logger.exception("failed to delete node [%s]: not found.", name)
            raise
        node.shutdown()
