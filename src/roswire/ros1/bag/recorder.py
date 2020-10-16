# -*- coding: utf-8 -*-
# http://wiki.ros.org/Bags/Format/2.0
__all__ = ('BagRecorder',)

import os
import shlex
import shutil
import threading
import time
from types import TracebackType
from typing import Optional, Type

import dockerblade
from loguru import logger

from ... import exceptions
from ...common import NodeManager


class BagRecorder:
    def __init__(self,
                 fn_dest: str,
                 ws_host: str,
                 shell: dockerblade.Shell,
                 nodes: NodeManager,
                 exclude_topics: Optional[str] = None,
                 restrict_to_topics: Optional[str] = None
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
        shell: Shell
            a shell proxy.
        nodes: NodeManager
            access to nodes for the associated ROS graph.
        exclude_topics: str, optional
            an optional regular expression specifying the topics that should
            be excluded from the bag.
        restrict_to_topics: str, optional
            An optional regular expression specifying the topics to which
            recording should be restricted.
        """
        self.__lock: threading.Lock = threading.Lock()
        self.__process: Optional[dockerblade.popen.Popen] = None
        self.__started: bool = False
        self.__stopped: bool = False
        self.__shell: dockerblade.Shell = shell
        self.__nodes: NodeManager = nodes
        self.__exclude_topics: Optional[str] = exclude_topics
        self.__restrict_to_topics: Optional[str] = restrict_to_topics

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

    def __enter__(self) -> 'BagRecorder':
        self.start()
        return self

    def __exit__(self,
                 ex_type: Optional[Type[BaseException]],
                 ex_val: Optional[BaseException],
                 ex_tb: Optional[TracebackType]
                 ) -> None:
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
            args = ['rosbag record', '-q',
                    f'-O {self.__fn_container}',
                    f'__name:={self.__bag_name}']
            if self.__exclude_topics:
                args += ['-x', shlex.quote(self.__exclude_topics)]
            if self.__restrict_to_topics:
                args += ['-e', shlex.quote(self.__restrict_to_topics)]
            else:
                args.append('-a')
            command = ' '.join(args)
            self.__process = self.__shell.popen(command,
                                                stderr=False,
                                                stdout=False)
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
                    logger.debug(f"bag file saved to {self.__fn_host_dest}")
                else:
                    logger.debug("bag file will not be saved")
                os.remove(self.__fn_host_temp)
            else:
                logger.debug("temporary bag file not found on "
                             f"host: {self.__fn_host_temp}")
        logger.debug("stopped bag recording")
