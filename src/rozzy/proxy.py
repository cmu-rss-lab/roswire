__all__ = ['ShellProxy', 'ROSProxy']

from typing import Tuple, Dict, Optional, Iterator, Any
import os
import xmlrpc.client
import logging
import time

from bugzoo import BugZoo as BugZooDaemon
from bugzoo import Container as BugZooContainer
from bugzoo.cmd import PendingExecResponse

from .exceptions import RozzyException

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class ShellProxy(object):
    """
    Provides shell access for a given BugZoo container.
    """
    def __init__(self,
                 daemon_bugzoo: BugZooDaemon,
                 container_bugzoo: BugZooContainer
                 ) -> None:
        self.__daemon_bugzoo = daemon_bugzoo
        self.__container_bugzoo = container_bugzoo

    def execute(self, command: str, **kwargs) -> Tuple[int, str, float]:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo, command, **kwargs)
        return r.code, r.output, r.duration

    def non_blocking_execute(self,
                             command: str,
                             **kwargs
                             ) -> PendingExecResponse:
        mgr = self.__daemon_bugzoo.containers
        r = mgr.command(self.__container_bugzoo, command, block=False, **kwargs)
        return r


class ParameterServerProxy(object):
    """
    See: http://wiki.ros.org/ROS/Parameter%20Server%20API
    """
    def __init__(self, connection: xmlrpc.client.ServerProxy) -> None:
        """
        Constructs a new parameter server proxy using an XML-RPC server proxy
        for a given ROS master.
        """
        self.__caller_id = '/rozzy'
        self.__connection = connection

    def __iter__(self) -> Iterator[str]:
        """
        Returns an iterator over the names of the parameters stored on the server.
        """
        conn = self.__connection
        code, msg, result = conn.getParamNames(self.__caller_id)
        if code != 1:
            raise RozzyException("bad API call!")
        yield from result

    def __getitem__(self, key: str) -> Any:
        """
        Fetches the value of a given parameter from the server.
        If the provided key is a namespace, then the contents of that
        namespace will be returned as a dictionary.

        Parameters:
            key: the name of the parameter (or namespace).

        Returns:
            The value of the parameter or the contents of the given namespace.

        Raises:
            KeyError: if no parameter with the given key is found on the
                parameter server.
        """
        conn = self.__connection
        code, msg, result = conn.getParam(self.__caller_id, key)
        # FIXME check for a specific code
        if code != 1:
            raise KeyError(key)
        return result

    def __delitem__(self, key: str) -> None:
        """
        Deletes a given parameter (or parameter tree) from the server.

        Parameters:
            key: the key for the parameter or parameter tree.

        Raises:
            KeyError: if no parameter or parameter tree is found with the
                given key on the server.
        """
        conn = self.__connection
        code, msg, result = conn.deleteParam(self.__caller_id, key)
        # FIXME check for a specific code
        if code != 1:
            raise KeyError(key)


class ROSProxy(object):
    """
    Provides access to a remote ROS master via XML-RPC.
    """
    def __init__(self,
                 shell: ShellProxy,
                 ip_address: str,
                 port: int = 11311
                 ) -> None:
        self.__shell = shell
        self.__caller_id = '/rozzy'
        self.__port = port
        self.__ip_address = ip_address
        self.__uri = "http://{}:{}".format(ip_address, port)
        logger.debug("connecting to ROS Master: %s", self.__uri)
        self.__connection = xmlrpc.client.ServerProxy(self.__uri)
        time.sleep(5)  # FIXME #1
        self.__parameters = ParameterServerProxy(self.__connection)

    # TODO ability to kill nodes
    @property
    def uri(self) -> str:
        """
        The URI of the ROS Master.
        """
        return self.__uri

    @property
    def parameters(self) -> ParameterServerProxy:
        """
        Provides access to the parameter server for this ROS Master.
        """
        return self.__parameters

    @property
    def connection(self) -> xmlrpc.client.ServerProxy:
        """
        The XML-RPC connection to the ROS master.
        """
        return self.__connection

    @property
    def topic_to_type(self) -> Dict[str, str]:
        conn = self.connection
        code, msg, result = conn.getTopicTypes(self.__caller_id)
        if code != 1:
            raise RozzyException("bad API call!")
        return {name: typ for (name, typ) in result}

    def lookup_node(self, name: str) -> Optional[str]:
        """
        Attempts to retrieve the XML-RPC URI of a given node.

        Parameters:
            name: the name of the node.

        Returns:
            the URI of the node if found, or None if not.
        """
        conn = self.connection
        code, msg, result = conn.lookupNode(self.__caller_id, name)
        if code != 1:
            return None
        # TODO convert URI to host network
        return result

    """
    def launch(self) -> None:
        pass

    def record(self) -> Iterator[ROSBagProxy]:
        pass

    def replay(self) -> None:
        pass
    """


# TODO ROSBagProxy


# TODO CoverageProxy
# - instrument: Python, C/C++
# - deinstrument
# - extract
# - flush
