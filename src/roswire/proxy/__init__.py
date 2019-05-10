# -*- coding: utf-8 -*-
__all__ = (
    'ShellProxy',
    'ServiceManagerProxy',
    'ParameterServerProxy',
    'NodeManagerProxy',
    'NodeProxy',
    'ROSProxy',
    'BagRecorderProxy',
    'BagPlayerProxy',
    'FileProxy',
    'ContainerProxy',
    'CatkinProxy',
    'CatkinToolsProxy',
    'CatkinMakeProxy',
    'ContainerProxyManager'
)

from typing import (Tuple, Dict, Optional, Iterator, Any, List, Union,
                    Collection)
import os
import xmlrpc.client
import shlex
import logging
import time

from .shell import ShellProxy
from .file import FileProxy
from .catkin import CatkinProxy, CatkinToolsProxy, CatkinMakeProxy
from .container import ContainerProxy, ContainerProxyManager
from .parameters import ParameterServerProxy
from .bag import BagRecorderProxy, BagPlayerProxy
from .node import NodeProxy, NodeManagerProxy
from .service import ServiceManagerProxy
from ..description import SystemDescription
from ..exceptions import ROSWireException

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class ROSProxy:
    """Provides access to a remote ROS master via XML-RPC."""
    def __init__(self,
                 description: SystemDescription,
                 shell: ShellProxy,
                 files: FileProxy,
                 ws_host: str,
                 ip_address: str,
                 port: int = 11311
                 ) -> None:
        self.__description = description
        self.__shell = shell
        self.__files = files
        self.__ws_host = ws_host
        self.__caller_id = '/roswire'
        self.__port = port
        self.__ip_address = ip_address
        self.__uri = f"http://{ip_address}:{port}"
        logger.debug("connecting to ROS Master: %s", self.__uri)
        self.__connection = xmlrpc.client.ServerProxy(self.__uri)
        time.sleep(5)  # FIXME #1
        self.__parameters = ParameterServerProxy(self.__connection)
        self.__nodes: NodeManagerProxy = \
            NodeManagerProxy(self.__ip_address,
                             self.__connection,
                             self.__shell)
        self.__services: ServiceManagerProxy = \
            ServiceManagerProxy(self.__description,
                                self.__ip_address,
                                self.__connection,
                                self.__shell)

    @property
    def uri(self) -> str:
        """The URI of the ROS Master."""
        return self.__uri

    @property
    def nodes(self) -> NodeManagerProxy:
        """Provides access to the nodes running on this ROS master."""
        return self.__nodes

    @property
    def services(self) -> ServiceManagerProxy:
        """Provides access to the services advertised on this ROS master."""
        return self.__services

    @property
    def parameters(self) -> ParameterServerProxy:
        """Provides access to the parameter server for this ROS Master."""
        return self.__parameters

    @property
    def connection(self) -> xmlrpc.client.ServerProxy:
        """The XML-RPC connection to the ROS master."""
        return self.__connection

    @property
    def topic_to_type(self) -> Dict[str, str]:
        conn = self.connection
        code, msg, result = conn.getTopicTypes(self.__caller_id)
        if code != 1:
            raise ROSWireException("bad API call!")
        return {name: typ for (name, typ) in result}

    def launch(self,
               filename: str,
               package: Optional[str] = None,
               *,
               args: Optional[Dict[str, Union[int, str]]] = None,
               prefix: Optional[str] = None
               ) -> None:
        """Provides an interface to roslaunch.

        Parameters
        ----------
        filename: str
            The name of the launch file (or an absolute path).
        package: str, optional
            The name of the package to which the launch file belongs.
        args: Dict[str, Union[int, str]], optional
            Keyword arguments that should be supplied to roslaunch.
        prefix: str, optional
            An optional prefix to add before the roslaunch command.
        """
        launch_args: List[str] = [f'{arg}:={val}' for arg, val in args.items()]
        cmd = ['roslaunch']
        if package:
            cmd += [shlex.quote(package)]
        cmd += [shlex.quote(filename)]
        cmd += launch_args
        cmd_str = ' '.join(cmd)
        self.__shell.non_blocking_execute(cmd_str)

    def record(self,
               fn: str,
               exclude_topics: Optional[Collection[str]] = None
               ) -> BagRecorderProxy:
        """Provides an interface to rosbag for recording ROS topics."""
        return BagRecorderProxy(fn,
                                self.__ws_host,
                                self.__shell,
                                self.__nodes,
                                exclude_topics=exclude_topics)

    def playback(self,
                 fn: str,
                 *,
                 file_on_host: bool = True
                 ) -> BagPlayerProxy:
        """Provides an interface to rosbag for replaying bag files."""
        fn_ctr: str
        delete_file_after_use: bool = False
        if file_on_host:
            if fn.startswith(self.__ws_host):
                fn_ctr = os.path.join('/.roswire', fn[len(self.__ws_host):])
            else:
                delete_file_after_use = True
                fn_ctr = self.__files.mktemp(suffix='.bag')
                logger.debug("copying bag from host [%s] to container [%s]",
                             fn, fn_ctr)
                self.__files.copy_from_host(fn, fn_ctr)
        else:
            fn_ctr = fn
        logger.debug("playing back bag file: %s", fn_ctr)
        return BagPlayerProxy(fn_ctr,
                              self.__shell,
                              self.__files,
                              delete_file_after_use=delete_file_after_use)
