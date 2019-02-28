__all__ = ['NodeManagerProxy', 'NodeProxy']

from typing import Iterator, Set
from urllib.parse import urlparse
import xmlrpc.client
import logging

from ..exceptions import RozzyException, NodeNotFoundError

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class NodeProxy:
    def __init__(self,
                 name: str,
                 url_host_network: str
                 ) -> None:
        """
        Constructs a proxy for a given name.

        Parameters:
            name: the name of the node.
            url_host_network: the URL of the node on the host network.
        """
        self.__name = name
        self.__url = url_host_network

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
        code, status, pid = self.api.getPid('/.rozzy')
        if code != 1:
            m = f"failed to obtain PID [{self.name}]: {status} (code: {code})"
            raise RozzyException(m)
        assert isinstance(pid, int)
        assert pid > 0
        return pid

    def shutdown(self, reason: str = '') -> None:
        code, status, ignore = self.api.shutdown()


class NodeManagerProxy:
    def __init__(self,
                 host_ip_master: str,
                 api: xmlrpc.client.ServerProxy
                 ) -> None:
        self.__host_ip_master = host_ip_master
        self.__api = api

    @property
    def api(self) -> xmlrpc.client.ServerProxy:
        return self.__api

    def __iter__(self) -> Iterator[str]:
        names = set()  # type: Set[str]
        code, status, state = self.api.getSystemState('./rozzy')
        for s in state:
            for t, l in s:
                names.update(n for n in l)
        yield from names

    def __getitem__(self, name: str) -> NodeProxy:
        """
        Attempts to fetch a given node.

        Raises:
            NodeNotFoundError: if there is no node with the given name.
        """
        code, status, uri_container = self.api.lookupNode('/.rozzy', name)
        if code == -1:
            raise NodeNotFoundError(name)
        if code != 1:
            m = f"unexpected error when attempting to find node [{name}]: {status} (code: {code})"   # noqa: pycodestyle
            raise RozzyException(m)

        # convert URI to host network
        port = urlparse(uri_container).port
        uri_host = f"http://{self.__host_ip_master}:{port}"
        return NodeProxy(name, uri_host)

    def __delitem__(self, name: str) -> None:
        try:
            node = self[name]
        except KeyError:
            logger.exception("failed to kill node [%s]: node not found.", name)
            raise
        node.shutdown()
