__all__ = ['NodeManagerProxy', 'NodeProxy']

from typing import Iterator, Set
import xmlrpc.client
import logging

from ..exceptions import RozzyException

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class NodeProxy:
    def __init__(self, name: str) -> None:
        self.__name = name
        self.__url = "FIXME"  # FIXME

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
        if code != 0:
            m = "bad API call: failed to obtain PID for node [{}]"
            m = m.format(self.name)
            raise RozzyException(m)
        assert isinstance(pid, int)
        assert pid > 0
        return pid

    def shutdown(self, reason: str = '') -> None:
        code, status, ignore = self.api.shutdown()


class NodeManagerProxy:
    def __init__(self, api: xmlrpc.client.ServerProxy) -> None:
        self.__api = api

    @property
    def api(self) -> xmlrpc.client.ServerProxy:
        return self.__api

    def __iter__(self) -> Iterator[str]:
        names = set()  # type: Set[str]
        code, status, state = api.getSystemState()
        for s in state:
            for t, l in s:
                names.update(n for n in l)
        yield from names

    def __getitem__(self, name: str) -> NodeProxy:
        code, status, uri = self.api.lookupNode('/.rozzy', name)
        # FIXME
        if code != 0:
            raise RozzyException
        # TODO convert URI to host network
        return

    def __delitem__(self, name: str) -> None:
        try:
            node = self[name]
        except KeyError:
            logger.exception("failed to kill node [%s]: node not found.", name)
            raise
        node.shutdown()
