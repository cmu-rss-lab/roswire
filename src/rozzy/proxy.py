__all__ = ['ShellProxy', 'ROSProxy']

from typing import Tuple, Dict
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

        # FIXME #1
        time.sleep(5)

        # self.__parameters = ParameterServerProxy()

    # TODO ability to kill nodes
    @property
    def uri(self) -> str:
        """
        The URI of the ROS Master.
        """
        return self.__uri

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

    """
    @property
    def parameters(self) -> ParameterServerProxy:
        return self.__parameters

        # publishers, subscribers, services
        # nodes

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
