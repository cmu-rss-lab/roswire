# -*- coding: utf-8 -*-
__all__ = ('ROS1',)

import os
import time
import typing
import xmlrpc.client
from typing import Dict, Optional, Sequence, Tuple

import dockerblade
from loguru import logger

from .bag import BagPlayer, BagRecorder
from .launch import ROS1LaunchManager
from .node_manager import ROS1NodeManager
from .parameter import ParameterServer
from .service import ServiceManager
from .state import SystemStateProbe
from ..common import NodeManager, ROSLaunchManager, SystemState
from ..exceptions import ROSWireException

if typing.TYPE_CHECKING:
    from .. import AppDescription


class ROS1:
    """Provides access to a remote ROS master via XML-RPC.

    Attributes
    ----------
    uri: str
        The URI of the ROS Master.
    connection: xmlrpc.client.ServerProxy
        The XML-RPC connection to the ROS Master.
    nodes: NodeManager
        Provides access to the nodes running on this ROS Master.
    state: SystemState
        The instantaneous state of the ROS Master.
    roslaunch: ROSLaunchManager
        Provides access to launch-related functionality.
    services: ServiceManager
        Provides access to the services advertised on this ROS Master.
    parameters: ParameterServer
        Provides access to the parameter server for this ROS Master.
    topic_to_type: Dict[str, str]
        A mapping from topic names to the names of their message types.
    """

    def __init__(self,
                 description: 'AppDescription',
                 shell: dockerblade.Shell,
                 files: dockerblade.FileSystem,
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
        logger.debug(f"connecting to ROS Master: {self.__uri}")
        self.__connection = xmlrpc.client.ServerProxy(self.__uri)
        time.sleep(5)  # FIXME #1
        self.__parameters = ParameterServer(self.__connection)
        self.__nodes: NodeManager = \
            ROS1NodeManager(self.__ip_address,
                            self.__connection,
                            self.__shell)
        self.__services: ServiceManager = \
            ServiceManager(self.__description,
                           self.__ip_address,
                           self.__connection,
                           self.__shell)
        self.__state_probe: SystemStateProbe = \
            SystemStateProbe.via_xmlrpc_connection(self.__connection)
        self.roslaunch: ROSLaunchManager = \
            ROS1LaunchManager(self.__shell, self.__files)

    @property
    def nodes(self) -> NodeManager:
        return self.__nodes

    @property
    def services(self) -> ServiceManager:
        return self.__services

    @property
    def parameters(self) -> ParameterServer:
        return self.__parameters

    @property
    def connection(self) -> xmlrpc.client.ServerProxy:
        return self.__connection

    @property
    def state(self) -> SystemState:
        return self.__state_probe()

    @property
    def topic_to_type(self) -> Dict[str, str]:
        code: int
        msg: str
        result: Sequence[Tuple[str, str]]
        code, msg, result = \
            self.connection.getTopicTypes(self.__caller_id)  # type: ignore
        if code != 1:
            raise ROSWireException("bad API call!")
        return {name: typ for (name, typ) in result}

    def record(self,
               fn: str,
               exclude_topics: Optional[str] = None,
               restrict_to_topics: Optional[str] = None
               ) -> BagRecorder:
        """Provides an interface to rosbag for recording ROS topics to disk.

        Note
        ----
        This method records bag files to the host machine, and not to the
        container where the ROS instance is running.

        Parameters
        ----------
        fn: str
            The name of the file, on the host machine, to which the bag should
            be recorded
        exclude_topics: str, optional
            An optional regular expression specifying the topics that should
            be excluded from the bag.
        restrict_to_topics: str, optional
            An optional regular expression specifying the topics to which
            recording should be restricted.

        Returns
        -------
        BagRecorder
            An interface for dynamically interacting with the bag recorder.
        """
        return BagRecorder(fn,
                           self.__ws_host,
                           self.__shell,
                           self.__nodes,
                           exclude_topics=exclude_topics,
                           restrict_to_topics=restrict_to_topics)

    def playback(self,
                 fn: str,
                 *,
                 file_on_host: bool = True
                 ) -> BagPlayer:
        """Provides an interface to rosbag for replaying bag files from disk.

        Parameters
        ----------
        fn: str
            The bag file that should be replayed.
        file_on_host: bool
            If :code:`True`, as by default, :code:`fn` will be considered to
            be a file on the host machine. If :code:`False`, :code:`fn` will
            be considered to be a file inside the container.

        Returns
        -------
        BagPlayer
            An interface for dynamically controlling the bag player.
        """
        fn_ctr: str
        delete_file_after_use: bool = False
        if file_on_host:
            if fn.startswith(self.__ws_host):
                fn_ctr = os.path.join('/.roswire', fn[len(self.__ws_host):])
            else:
                delete_file_after_use = True
                fn_ctr = self.__files.mktemp(suffix='.bag')
                logger.debug(f"copying bag from host [{fn}] "
                             f"to container [{fn_ctr}]")
                self.__files.copy_from_host(fn, fn_ctr)
        else:
            fn_ctr = fn
        logger.debug(f"playing back bag file: {fn_ctr}")
        return BagPlayer(fn_ctr,
                         self.__shell,
                         self.__files,
                         delete_file_after_use=delete_file_after_use)
