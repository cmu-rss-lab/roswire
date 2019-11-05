# -*- coding: utf-8 -*-
# http://wiki.ros.org/Bags/Format/2.0
__all__ = ('BagRecorderProxy', 'BagPlayerProxy')

from typing import Optional, Collection
import logging
import shutil
import time
import pathlib
import threading
import subprocess
import os

from .file import FileProxy
from .shell import ShellProxy, Popen
from .node import NodeManagerProxy
from .. import exceptions

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class BagPlayerProxy:
    def __init__(self,
                 fn_container: str,
                 shell: ShellProxy,
                 files: FileProxy,
                 *,
                 delete_file_after_use: bool = False
                 ) -> None:
        self.__lock = threading.Lock()
        self.__fn_container = fn_container
        self.__shell = shell
        self.__files = files
        self.__delete_file_after_use = delete_file_after_use
        self.__started = False
        self.__stopped = False
        self._process: Optional[Popen] = None

    @property
    def started(self) -> bool:
        """Indicates whether or not playback has started."""
        return self.__started

    @property
    def stopped(self) -> bool:
        """Indicates whether or not playback has stopped."""
        return self.__stopped

    def __enter__(self) -> 'BagPlayerProxy':
        self.start()
        return self

    def __exit__(self, ex_type, ex_val, ex_tb) -> None:
        if ex_type is not None:
            logger.error("error occurred during bag playback",
                         exc_info=(ex_type, ex_val, ex_tb))
        if not self.stopped:
            self.stop()

    def finished(self) -> bool:
        """Checks whether playback has completed."""
        p = self._process
        return p.finished if p else False

    def wait(self, time_limit: Optional[float] = None) -> None:
        """Blocks until playback has finished.

        Parameters
        ----------
        time_limit: Optional[float] = None
            an optional time limit.

        Raises
        ------
        PlayerTimeout:
            if playback did not finish within the provided timeout.
        PlayerFailure:
            if an unexpected occurred during playback.
        """
        assert self._process
        try:
            self._process.wait(time_limit)
            retcode = self._process.returncode
            assert retcode is not None
            if retcode != 0:
                out = '\n'.join(self._process.stream)
                raise exceptions.PlayerFailure(retcode, out)
        except subprocess.TimeoutExpired:
            raise exceptions.PlayerTimeout

    def start(self) -> None:
        """Starts playback from the bag.

        Raises
        ------
        PlayerAlreadyStarted:
            if the player has already started.
        """
        logger.debug("starting bag playback")
        with self.__lock:
            if self.__started:
                raise exceptions.PlayerAlreadyStarted
            self.__started = True
            cmd: str = f"rosbag play -q {self.__fn_container}"
            self._process = self.__shell.popen(cmd)
            logger.debug("started bag playback")

    def stop(self) -> None:
        """Stops playback from the bag.

        Raises:
            PlayerAlreadyStopped: if the player has already been stopped.
        """
        logger.debug("stopping bag playback")
        with self.__lock:
            if self.__stopped:
                raise exceptions.PlayerAlreadyStopped
            if not self.__started:
                raise exceptions.PlayerNotStarted
            assert self._process
            self._process.kill()
            out = '\n'.join(list(self._process.stream))
            logger.debug("player output:\n%s", out)
            self._process = None
            if self.__delete_file_after_use:
                self.__files.remove(self.__fn_container)
            self.__stopped = True
        logger.debug("stopped bag playback")


class BagRecorderProxy:
    def __init__(self,
                 fn_dest: str,
                 ws_host: str,
                 shell: ShellProxy,
                 nodes: NodeManagerProxy,
                 exclude_topics: Optional[Collection[str]] = None
                 ) -> None:
        """
        Notes
        -----
            This object should not be constructed directly.

        Parameters
        ----------
        fn_dest: str
            the destination filepath for the bag (on the host).
        ws_host: str
            the workspace directory for the associated container (on the host).
        shell: ShellProxy
            a shell proxy.
        nodes: NodeManagerProxy
            access to nodes for the associated ROS graph.
        excluded_topics: Optional[Collection[str]] = None
            an optional list of topics that should be excluded from the bag.
        """
        self.__lock: threading.Lock = threading.Lock()
        self.__process: Optional[Popen] = None
        self.__started: bool = False
        self.__stopped: bool = False
        self.__shell: ShellProxy = shell
        self.__nodes: NodeManagerProxy = nodes

        # FIXME generate a bag name
        self.__bag_name: str = "my_bag"

        # create a temporary file inside the shared directory
        self.__fn_host_dest: str = fn_dest
        self.__fn_container: str = f"/.roswire/{self.__bag_name}.bag"
        self.__fn_host_temp: str = \
            os.path.join(ws_host, f'{self.__bag_name}.bag')

    @property
    def started(self) -> bool:
        """Indicates whether or not recording has started."""
        return self.__started

    @property
    def stopped(self) -> bool:
        """Indicates whether or not recording has stopped."""
        return self.__stopped

    def __enter__(self) -> 'BagRecorderProxy':
        self.start()
        return self

    def __exit__(self, ex_type, ex_val, ex_tb) -> None:
        if ex_type is not None:
            logger.error("error occurred during bag recording",
                         exc_info=(ex_type, ex_val, ex_tb))
        should_save = ex_type is None
        if not self.stopped:
            self.stop(save=should_save)

    def start(self) -> None:
        """Starts recording to the bag.

        Raises
        ------
        RecorderAlreadyStarted:
            if the recorder has already been started.
        """
        logger.debug("starting bag recording")
        with self.__lock:
            if self.__started:
                raise exceptions.RecorderAlreadyStarted
            self.__started = True
            cmd: str = ("rosbag record -q -a"
                        f" -O {self.__fn_container}"
                        f" __name:={self.__bag_name}")
            self.__process = self.__shell.popen(cmd)
            logger.debug("started bag recording")

    def stop(self, save: bool = True) -> None:
        """Stops recording to the bag.

        Parameters
        ----------
        save: bool
            specifies whether the bag file should be saved to disk.

        Raises
        ------
        RecorderAlreadyStopped:
            if this recorder has already been stopped.
        """
        logger.debug("stopping bag recording")
        with self.__lock:
            if self.__stopped:
                raise exceptions.RecorderAlreadyStopped
            if not self.__started:
                raise exceptions.RecorderNotStarted

            name_node = f'/{self.__bag_name}'
            del self.__nodes[name_node]
            time.sleep(5)  # FIXME

            assert self.__process
            self.__process.kill()

            if os.path.exists(self.__fn_host_temp):
                if save:
                    shutil.copyfile(self.__fn_host_temp, self.__fn_host_dest)
                    logger.debug("bag file saved to %s", self.__fn_host_dest)
                else:
                    logger.debug("bag file will not be saved")
                os.remove(self.__fn_host_temp)
            else:
                logger.debug("temporary bag file not found on host: %s",
                             self.__fn_host_temp)
        logger.debug("stopped bag recording")
