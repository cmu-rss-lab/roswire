__all__ = ['NodeManagerProxy', 'NodeProxy']

from typing import Iterator, Set, Mapping
from urllib.parse import urlparse
import xmlrpc.client
import logging

from .shell import ShellProxy
from ..exceptions import ROSWireException, NodeNotFoundError

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class NodeProxy:
    def __init__(self,
                 name: str,
                 url_host_network: str,
                 shell: ShellProxy
                 ) -> None:
        """
        Constructs a proxy for a given name.

        Parameters:
            name: the name of the node.
            url_host_network: the URL of the node on the host network.
            shell: a shell proxy.
        """
        self.__name = name
        self.__url = url_host_network
        self.__shell = shell

    @property
    def api(self) -> xmlrpc.client.ServerProxy:
        """
        Provides access to the XML-RPC API for this node.
        """
        return xmlrpc.client.ServerProxy(self.url)

    @property
    def name(self) -> str:
        """
        The fully qualified name of this node.
        """
        return self.__name

    @property
    def url(self) -> str:
        """
        The URL that should be used to access this node from the host network.
        """
        return self.__url

    @property
    def pid(self) -> int:
        """
        The PID of the main process for this node.
        """
        code, status, pid = self.api.getPid('/.roswire')
        if code != 1:
            m = f"failed to obtain PID [{self.name}]: {status} (code: {code})"
            raise ROSWireException(m)
        assert isinstance(pid, int)
        assert pid > 0
        return pid

    def shutdown(self) -> None:
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
        """
        Attempts to fetch a given node.

        Raises:
            NodeNotFoundError: if there is no node with the given name.
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
