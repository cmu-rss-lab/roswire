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

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


class BagRecorderProxy:
    def __init__(self,
                 ws_host: str,
                 shell: ShellProxy,
                 nodes: NodeManagerProxy,
                 excluded_topics: Optional[Collection[str]] = None
                 ) -> None:
        """
        Parameters:
            shell: a shell proxy.
            excluded_topics: an optional list of topics that should be excluded
                from the bag.
        """
        self.__stopped: bool = False
        self.__shell: ShellProxy = shell
        self.__nodes: NodeManagerProxy = nodes

        # FIXME generate a bag name
        self.__bag_name: str = "my_bag"

        # create a temporary file inside the shared directory
        self.__fn_container: str = f"/.rozzy/{self.__bag_name}.bag"
        self.__fn_host: str = os.path.join(ws_host, f'{self.__bag_name}.bag')
        # pathlib.Path(self.__fn_host).touch()

        # launch rosbag process
        # FIXME bad mounting?!
        cmd: str = ("rosbag record -q -a"
                    f" -O {self.__fn_host}"
                    f" __name:={self.__bag_name}")
        self.__shell.non_blocking_execute(cmd)

    @property
    def stopped(self) -> bool:
        return self.__stopped

    def __enter__(self) -> 'BagRecorderProxy':
        return self

    def __exit__(self) -> None:
        if not self.stopped:
            self.stop()
        if os.path.exists(self.__fn_host):
            os.remove(self.__fn_host)

    def stop(self) -> None:
        """
        Stops recording to the bag.
        """
        if not self.stopped:
            name_node = f'/{self.__bag_name}'
            del self.__nodes[name_node]
            time.sleep(10)

    def save(self, fn: str) -> None:
        """
        Saves the contents of the bag to a given file on the host machine.
        """
        shutil.move(self.__fn_host, fn)
