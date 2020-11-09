# -*- coding: utf-8 -*-
# http://wiki.ros.org/Bags/Format/2.0
__all__ = ('BagPlayer',)

import subprocess
import threading
from types import TracebackType
from typing import Optional, Type

import dockerblade
from loguru import logger

from ... import exceptions


class BagPlayer:
    def __init__(self,
                 fn_container: str,
                 shell: dockerblade.Shell,
                 files: dockerblade.FileSystem,
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
        self._process: Optional[dockerblade.popen.Popen] = None

    @property
    def started(self) -> bool:
        """Indicates whether or not playback has started."""
        return self.__started

    @property
    def stopped(self) -> bool:
        """Indicates whether or not playback has stopped."""
        return self.__stopped

    def __enter__(self) -> 'BagPlayer':
        self.start()
        return self

    def __exit__(self,
                 ex_type: Optional[Type[BaseException]],
                 ex_val: Optional[BaseException],
                 ex_tb: Optional[TracebackType]
                 ) -> None:
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
                out = '\n'.join(self._process.stream)  # type: ignore
                raise exceptions.PlayerFailure(retcode, out)
        except subprocess.TimeoutExpired as error:
            raise exceptions.PlayerTimeout from error

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
            command: str = f"rosbag play -q {self.__fn_container}"
            self._process = self.__shell.popen(command,
                                               stdout=False,
                                               stderr=False)
            logger.debug('started bag playback')

    def stop(self) -> None:
        """Stops playback from the bag.

        Raises
        ------
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
            out = '\n'.join(list(self._process.stream))  # type: ignore
            logger.debug("player output:\n%s", out)
            self._process = None
            if self.__delete_file_after_use:
                self.__files.remove(self.__fn_container)
            self.__stopped = True
        logger.debug("stopped bag playback")
