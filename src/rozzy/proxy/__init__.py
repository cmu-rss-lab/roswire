__all__ = ['ShellProxy', 'ParameterServerProxy', 'ROSProxy', 'ROSBagProxy']

from typing import Tuple, Dict, Optional, Iterator, Any
import os
import xmlrpc.client
import logging
import time

from .shell import ShellProxy
from .parameters import ParameterServerProxy
from .rosbag import ROSBagProxy
from ..exceptions import RozzyException

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


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

    def bag(self) -> Iterator[ROSBagProxy]:
        p = ROSBagProxy()
        yield p
        # ensure that the file inside the container is destroyed

    @property
    def topic_to_type(self) -> Dict[str, str]:
        conn = self.connection
        code, msg, result = conn.getTopicTypes(self.__caller_id)
        if code != 1:
            raise RozzyException("bad API call!")
        return {name: typ for (name, typ) in result}

    """
    def launch(self, fn_launch: str) -> None:
        cmd = "roslaunch '{}'".format(fn_launch)
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
