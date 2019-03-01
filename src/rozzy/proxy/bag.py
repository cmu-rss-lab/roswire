# http://wiki.ros.org/Bags/Format/2.0
__all__ = ['BagRecorderProxy']

from typing import Optional, Collection
import logging
import shutil
import time
import pathlib
import os

from .shell import ShellProxy
from .node import NodeManagerProxy
from .. import exceptions

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class BagRecorderProxy:
    def __init__(self,
                 fn_dest: str,
                 ws_host: str,
                 shell: ShellProxy,
                 nodes: NodeManagerProxy,
                 excluded_topics: Optional[Collection[str]] = None
                 ) -> None:
        """
        Note:
            This object should not be constructed directly.

        Parameters:
            fn_dest: the destination filepath for the bag (on the host).
            ws_host: the workspace directory for the associated container (on
                the host).
            shell: a shell proxy.
            nodes: access to nodes for the associated ROS graph.
            excluded_topics: an optional list of topics that should be excluded
                from the bag.
        """
        self.__started: bool = False
        self.__stopped: bool = False
        self.__shell: ShellProxy = shell
        self.__nodes: NodeManagerProxy = nodes

        # FIXME generate a bag name
        self.__bag_name: str = "my_bag"

        # create a temporary file inside the shared directory
        self.__fn_host_dest: str = fn_dest
        self.__fn_container: str = f"/.rozzy/{self.__bag_name}.bag"
        self.__fn_host_temp: str = \
            os.path.join(ws_host, f'{self.__bag_name}.bag')

    @property
    def started(self) -> bool:
        """
        Indicates whether or not recording has started.
        """
        return self.__started

    @property
    def stopped(self) -> bool:
        """
        Indicates whether or not recording has stopped.
        """
        return self.__stopped

    def __enter__(self) -> 'BagRecorderProxy':
        self.start()
        return self

    def __exit__(self, ex_type, ex_val, ex_tb) -> None:
        # FIXME did an exception occur? if so, don't save.
        if not self.stopped:
            self.stop()
        if os.path.exists(self.__fn_host):
            os.remove(self.__fn_host)

    def start(self) -> None:
        """
        Starts recording to the bag.

        Raises:
            RecorderAlreadyStarted: if the recorder has already been started.
        """
        if self.started:
            raise exceptions.RecorderAlreadyStarted

        # TODO could acquire lock?
        self.__started = True

        # FIXME bad mounting?!
        cmd: str = ("rosbag record -q -a"
                    f" -O {self.__fn_host_temp}"
                    f" __name:={self.__bag_name}")
        self.__shell.non_blocking_execute(cmd)

    def stop(self) -> None:
        """
        Stops recording to the bag.

        Raises:
            RecorderAlreadyStopped: if this recorder has already been stopped.
        """
        if self.stopped:
            raise exceptions.RecorderAlreadyStopped

        name_node = f'/{self.__bag_name}'
        del self.__nodes[name_node]
        time.sleep(10)

        # FIXME exception handling
        shutil.copyfile(self.__fn_host_temp, self.__fn_host_dest)
        os.remove(self.__fn_host_temp)
