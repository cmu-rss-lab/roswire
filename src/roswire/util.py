# -*- coding: utf-8 -*-
__all__ = (
    "is_port_open",
    "Stopwatch",
    "tuple_from_iterable",
    "wait_till_open",
)

import contextlib
import socket
import time
import warnings
from timeit import default_timer as timer
from typing import Any, Iterable, Tuple

from . import exceptions as exc


def tuple_from_iterable(val: Iterable[Any]) -> Tuple[Any, ...]:
    """
    Builds a tuple from an iterable.

    Workaround for https://github.com/python-attrs/attrs/issues/519
    """
    return tuple(val)


class Stopwatch:
    def __init__(self) -> None:
        """Constructs a new, paused timer."""
        self.__offset: float = 0.0
        self.__paused: bool = True
        self.__time_start: float = 0.0

    def stop(self) -> None:
        """Freezes the timer."""
        if not self.__paused:
            self.__offset += timer() - self.__time_start
            self.__paused = True

    def start(self) -> None:
        """Resumes the timer."""
        if self.__paused:
            self.__time_start = timer()
            self.__paused = False
        else:
            warnings.warn("timer is already running")

    def reset(self) -> None:
        """Resets and freezes the timer."""
        self.__offset = 0.0
        self.__paused = True

    @property
    def paused(self) -> bool:
        """Returns True if this stopwatch is paused, or False if not."""
        return self.__paused

    @property
    def duration(self) -> float:
        """The number of seconds that the stopwatch has been running."""
        d = self.__offset
        if not self.__paused:
            d += timer() - self.__time_start
        return d


def wait_till_open(host: str,
                   port: int,
                   timeout: float,
                   *,
                   interval: float = 0.5
                   ) -> None:
    """Blocks until either a given port is open or a timeout expires.

    Parameters
    ----------
    host: str
        The name or IP address of the port host.
    port: int
        The port number.
    timeout: float
        The maximum number of seconds to wait before throwing a timeout.
    interval: float
        The number of seconds to wait between re-checking if the port is open.

    Raises
    ------
    TimeoutExpiredError
        If the timeout expires before the port is open.
    """
    with contextlib.closing(
        socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ) as s:
        stopwatch = Stopwatch()
        stopwatch.start()
        while stopwatch.duration < timeout:
            if s.connect_ex((host, port)) == 0:
                return
            time.sleep(interval)
    m = f"unable to reach port [{host}:{port}] after {timeout} seconds"
    raise exc.TimeoutExpiredError(m)


def is_port_open(host: str, port: int) -> bool:
    """Determines whether a port on a given host is open."""
    with contextlib.closing(
        socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ) as s:
        return s.connect_ex((host, port)) == 0
