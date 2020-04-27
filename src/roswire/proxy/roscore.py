# -*- coding: utf-8 -*-
__all__ = ('ROSCore',)

from typing import Dict, List, Mapping, Optional, Union
import os
import xmlrpc.client
import shlex
import time

from loguru import logger
import dockerblade

from .bag import BagRecorder, BagPlayer
from ..description import SystemDescription
from ..exceptions import ROSWireException
from .node import NodeManager
from .parameters import ParameterServer
from .service import ServiceManager


class ROSCore:
    """Provides access to a remote ROS master via XML-RPC.

    Attributes
    ----------
    uri: str
        The URI of the ROS Master.
    connection: xmlrpc.client.ServerProxy
        The XML-RPC connection to the ROS master.
    nodes: NodeManager
        Provides access to the nodes running on this ROS Master.
    services: ServiceManager
        Provides access to the services advertised on this ROS Master.
    parameters: ParameterServer
        Provides access to the parameter server for this ROS Master.
    topic_to_type: Dict[str, str]
        A mapping from topic names to the names of their message types.
    """
    def __init__(self,
                 description: SystemDescription,
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
            NodeManager(self.__ip_address,
                        self.__connection,
                        self.__shell)
        self.__services: ServiceManager = \
            ServiceManager(self.__description,
                           self.__ip_address,
                           self.__connection,
                           self.__shell)

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
    def topic_to_type(self) -> Dict[str, str]:
        conn = self.connection
        code, msg, result = conn.getTopicTypes(self.__caller_id)
        if code != 1:
            raise ROSWireException("bad API call!")
        return {name: typ for (name, typ) in result}

    def launch(self,
               filename: str,
               *,
               package: Optional[str] = None,
               args: Optional[Dict[str, Union[int, str]]] = None,
               prefix: Optional[str] = None,
               launch_prefixes: Optional[Mapping[str, str]] = None
               ) -> None:
        """Provides an interface to roslaunch.

        Parameters
        ----------
        filename: str
            The name of the launch file, or an absolute path to the launch
            file inside the container.
        package: str, optional
            The name of the package to which the launch file belongs.
        args: Dict[str, Union[int, str]], optional
            Keyword arguments that should be supplied to roslaunch.
        prefix: str, optional
            An optional prefix to add before the roslaunch command.
        launch_prefixes: Mapping[str, str], optional
            An optional mapping from nodes, given by their names, to their
            individual launch prefix.
        """
        shell = self.__shell
        if not args:
            args = {}
        if not launch_prefixes:
            launch_prefixes = {}
        launch_args: List[str] = [f'{arg}:={val}' for arg, val in args.items()]

        if launch_prefixes:
            m = "individual launch prefixes are not yet implemented"
            raise NotImplementedError(m)

        # determine the absolute path of the launch file
        if package:
            filename_original = filename
            logger.debug(f'determing location of launch file [{filename}]'
                         f' in package [{package}]')
            package_escaped = shlex.quote(package)
            find_package_command = f'rospack find {package_escaped}'
            package_path = shell.check_output(find_package_command,
                                              stderr=False)
            filename = os.path.join(package_path, 'launch', filename)
            logger.debug('determined location of launch file'
                         f' [{filename_original}] in package [{package}]: '
                         f'{filename}')

        cmd = ['roslaunch', shlex.quote(filename)]
        cmd += launch_args
        if prefix:
            cmd = [prefix] + cmd
        cmd_str = ' '.join(cmd)
        self.__shell.popen(cmd_str, stdout=False, stderr=False)

    def record(self,
               fn: str,
               exclude_topics: Optional[str] = None
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

        Returns
        -------
        BagRecorder
            An interface for dynamically interacting with the bag recorder.
        """
        return BagRecorder(fn,
                           self.__ws_host,
                           self.__shell,
                           self.__nodes,
                           exclude_topics=exclude_topics)

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
