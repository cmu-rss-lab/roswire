# -*- coding: utf-8 -*-
__all__ = ("ROS1",)

import os
import time
import typing
import xmlrpc.client
from types import TracebackType
from typing import (
    Dict,
    Mapping,
    Optional,
    Sequence,
    Tuple,
    Type,
)

import dockerblade
from loguru import logger

from .bag import BagPlayer, BagRecorder
from .launch import ROS1LaunchManager
from .node_manager import ROS1NodeManager
from .parameter import ParameterServer
from .service import ServiceManager
from .source import ROS1PackageSourceExtractor
from .state import SystemStateProbe
from .. import exceptions as exc
from ..common import (
    CMakeTarget,
    NodeManager,
    Package, ROSLaunchManager,
    SystemState,
)
from ..exceptions import ROSWireException
from ..util import is_port_open, Stopwatch, wait_till_open

if typing.TYPE_CHECKING:
    from .. import AppDescription


class ROS1:
    """Provides access to the ROS1 API.

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

    def __init__(
        self,
        description: "AppDescription",
        shell: dockerblade.Shell,
        files: dockerblade.FileSystem,
        ws_host: Optional[str],
        ip_address: str,
        port: int = 11311,
    ) -> None:
        self.__description = description
        self.__shell = shell
        self.__files = files
        self.__ws_host = ws_host
        assert port > 1023
        self.__port = port
        self.__ip_address = ip_address
        self.__uri = f"http://{ip_address}:{port}"
        self.__connection: Optional[xmlrpc.client.ServerProxy] = None
        self.__roscore_process: Optional[dockerblade.popen.Popen] = None
        self.__package_source_extractor = \
            ROS1PackageSourceExtractor.for_filesystem(self.__files)

    def __enter__(self) -> "ROS1":
        """
        Ensures that a ROS Master is up and running by either connecting to
        an existing ROS Master or launching a new one at the associated URI.
        """
        if not self.connected:
            self.connect_or_launch()
        return self

    def __exit__(
        self,
        ex_type: Optional[Type[BaseException]],
        ex_val: Optional[BaseException],
        ex_tb: Optional[TracebackType],
    ) -> None:
        """
        Disconnects from the associated ROS Master, and, if the ROS Master was
        launched by this class, kills the associated roscore process.
        """
        self.close()

    @property
    def connected(self) -> bool:
        """Indicates if there is a connection to the ROS Master."""
        return self.__connection is not None

    def close(self) -> None:
        logger.debug(f"closing connection to ROS Master: {self.__uri}")
        if self.__roscore_process is not None:
            self._shutdown()
        else:
            self._disconnect()

    def _shutdown(self) -> None:
        if self.__roscore_process is None:
            raise exc.IllegalOperation("No associated roscore process.")

        logger.debug("shutting down roscore...")
        self.__roscore_process.terminate()
        self.__roscore_process.wait(2.0)
        self.__roscore_process.kill()
        self.__roscore_process = None
        logger.debug("shutdown roscore")

        self._disconnect()

    def _disconnect(self) -> None:
        """Disconnects from the associated ROS Master.

        Raises
        ------
        NotConnectedError
            No connection to the ROS Master has been established.
        IllegalOperation
            If the ROS Master is managed by this class, then the associated
            roscore process must be killed before attempting to disconnect.
        """
        if self.__connection is None:
            raise exc.NotConnectedError("Not connected to ROS Master")

        if self.__roscore_process is not None:
            raise exc.IllegalOperation("Not connected to ROS Master")

        self.__connection = None
        logger.debug("disconnected from ROS Master")

    def connect_or_launch(self) -> None:
        """
        Attempts to connect to an already running ROS Master at the port
        associated with this interface. If ROS Master isn't running at that
        port, the roscore process will be launched.
        """
        if self._is_rosmaster_online():
            self.connect()
        else:
            self.launch()

    def connect(self, *, timeout: float = 30.0) -> None:
        """Establishes a connection to an already running ROS Master.

        Parameters
        ----------
        timeout: float
            The maximum number of seconds to wait when trying to connect to
            the ROS Master before timing out.

        Raises
        ------
        TimeoutExpiredError
            if a timeout occurred when attempting to connect to ROS Master.
        AlreadyConnectedError
            if a connection to the ROS Master has already been established.
        """
        logger.debug(f"attempting to connect to ROS Master: {self.__uri}")
        timer = Stopwatch()
        timer.start()
        if self.connected:
            raise exc.AlreadyConnectedError()

        logger.debug(f"waiting for ROS Master to be online: {self.__uri}")
        self._wait_until_rosmaster_is_online(timeout)
        logger.debug(f"ROS Master is online: {self.__uri}")

        self.__connection = xmlrpc.client.ServerProxy(self.__uri)
        self.__parameters = ParameterServer(self.__connection)
        self.__nodes: NodeManager = ROS1NodeManager(
            self.__ip_address, self.__connection, self.__shell
        )
        self.__services: ServiceManager = ServiceManager(
            self.__description,
            self.__ip_address,
            self.__connection,
            self.__shell,
        )
        self.__state_probe: SystemStateProbe = (
            SystemStateProbe.via_xmlrpc_connection(self.__connection)
        )
        self.roslaunch: ROSLaunchManager = ROS1LaunchManager(
            self.__shell, self.__files
        )

        self.__package_source_extractor = ROS1PackageSourceExtractor(
            self.__files
        )

        logger.debug("waiting for /rosout to be online")
        self._wait_until_rosout_is_online(timeout)
        logger.debug("/rosout is up and running")
        logger.debug(f"connected to ROS Master: {self.__uri}")

    def _wait_until_rosout_is_online(self, timeout: float) -> None:
        assert self.__connection is not None
        timer = Stopwatch()
        timer.start()
        is_online = False
        while not is_online:
            state = self.state
            is_online = "/rosout" in state.topics
            is_online &= "/rosout/set_logger_level" in state.services
            is_online &= "/rosout/set_logger_level" in state.services

            if timer.duration > timeout:
                m = "timed out waiting for /rosout to become available"
                raise exc.TimeoutExpiredError(m)

            time.sleep(0.1)

    def _wait_until_rosmaster_is_online(self, timeout: float = 5.0) -> None:
        """Blocks until the ROS Master is online or a timeout occurs.

        Parameters
        ----------
        timeout: float
            The maximum number of seconds to wait before timing out.

        Raises
        ------
        TimeoutExpiredError
            If the ROS Master does not become available within the given
            timeout.
        """
        wait_till_open(self.__ip_address, self.__port, timeout)

    def _is_rosmaster_online(self) -> bool:
        """
        Checks whether the ROS Master is online at the port associated with
        this interface.
        """
        return is_port_open(self.__ip_address, self.__port)

    def launch(self) -> None:
        """Launches the roscore process.

        Raises
        ------
        AlreadyConnected
            If a connection to ROS Master has already been established.
        """
        if self.__roscore_process is not None:
            raise exc.AlreadyConnectedError()

        command = f"roscore -p {self.__port}"
        logger.debug(f"launching roscore via: {command}")
        self.__roscore_process = self.__shell.popen(command)
        logger.debug("launched roscore")
        self.connect()

    def must_be_connected(self) -> None:
        """Raises a NotConnectedError if no connected to ROS Master."""
        if not self.connected:
            raise exc.NotConnectedError("not connected to ROS Master")

    @property
    def nodes(self) -> NodeManager:
        self.must_be_connected()
        return self.__nodes

    @property
    def services(self) -> ServiceManager:
        self.must_be_connected()
        return self.__services

    @property
    def parameters(self) -> ParameterServer:
        self.must_be_connected()
        return self.__parameters

    @property
    def connection(self) -> xmlrpc.client.ServerProxy:
        self.must_be_connected()
        assert self.__connection is not None
        return self.__connection

    @property
    def state(self) -> SystemState:
        self.must_be_connected()
        return self.__state_probe()

    @property
    def topic_to_type(self) -> Dict[str, str]:
        self.must_be_connected()

        code: int
        msg: str
        result: Sequence[Tuple[str, str]]
        # fmt: off
        code, msg, result = \
            self.connection.getTopicTypes('/roswire')  # type: ignore
        # fmt: on
        if code != 1:
            raise ROSWireException("bad API call!")
        return {name: typ for (name, typ) in result}

    def record(
        self,
        filename: str,
        exclude_topics: Optional[str] = None,
        restrict_to_topics: Optional[str] = None,
    ) -> BagRecorder:
        """Provides an interface to rosbag for recording ROS topics to disk.

        Note
        ----
        This method records bag files to the host machine, and not to the
        container where the ROS instance is running.

        Parameters
        ----------
        filename: str
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
        self.must_be_connected()
        return BagRecorder(
            filename,
            self.__ws_host,
            self.__shell,
            self.__nodes,
            exclude_topics=exclude_topics,
            restrict_to_topics=restrict_to_topics,
        )

    def playback(
        self,
        filename: str,
        *,
        file_on_host: bool = True
    ) -> BagPlayer:
        """Provides an interface to rosbag for replaying bag files from disk.

        Parameters
        ----------
        filename: str
            The bag file that should be replayed.
        file_on_host: bool
            If ``True``, as by default, ``filename`` will be considered to
            be a file on the host machine. If ``False``, ``filename`` will
            be considered to be a file inside the container.

        Returns
        -------
        BagPlayer
            An interface for dynamically controlling the bag player.
        """
        self.must_be_connected()

        container_filename: str
        delete_file_after_use = False

        # check if file is inside shared workspace
        if file_on_host and self.__ws_host and filename.startswith(self.__ws_host):  # noqa: E501
            container_filename = os.path.join(
                "/.roswire", filename[len(self.__ws_host):]
            )
        elif file_on_host:
            delete_file_after_use = True
            container_filename = self.__files.mktemp(suffix=".bag")
            logger.debug(
                f"copying bag from host [{filename}] "
                f"to container [{container_filename}]"
            )
            self.__files.copy_from_host(filename, container_filename)
        else:
            container_filename = filename

        logger.debug(f"playing back bag file: {container_filename}")
        return BagPlayer(
            container_filename,
            shell=self.__shell,
            files=self.__files,
            delete_file_after_use=delete_file_after_use,
        )

    def package_node_sources(
        self,
        package: Package,
    ) -> Mapping[str, CMakeTarget]:
        """
        Extracts the node -> source files mapping for the package with the
        source in ``package_path''

        Parameters
        ----------
        package: Package
            The package in the container filesystem that contains the package
            source

        Returns
        -------
        Mapping[str, NodeSourceInfo]
            A (possibly empty) mapping between node names provided by the
            package and their source information
        """
        return self.__package_source_extractor.get_cmake_info(
            package
        ).targets
