# -*- coding: utf-8 -*-
__all__ = (
    "is_port_open",
    "Stopwatch",
    "tuple_from_iterable",
    "wait_till_open",
)

import contextlib
import re
import socket
import time
import typing as t
import warnings
import xml.etree.ElementTree as ET  # noqa
from timeit import default_timer as timer

from . import exceptions as exc


def tuple_from_iterable(val: t.Iterable[t.Any]) -> t.Tuple[t.Any, ...]:
    """
    Builds a tuple from an iterable.

    Workaround for https://github.com/python-attrs/attrs/issues/519
    """
    return tuple(val)


def key_val_list_to_dict(key_values: t.List[str]) -> t.Dict[str, str]:
    """Converts a list of key, val pairs into a dict.

    Parameters
    ----------
    key_values: List[str]
        List of alternating key, value pairs in a list

    Returns
    -------
    Dict[str, str]
        A dictionary of key, value entries
    """
    assert len(key_values) % 2 == 0
    key_list = key_values[::2]
    value_list = key_values[1::2]
    return dict(zip(key_list, value_list))


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

def safer_xml_from_string(contents: str,
                          root_tag: str) -> ET.Element:
    """
    Tries to safely parse an XML string. To avoid some of
    the idiosyncracies of ROS, we strip out everything before the
    root tag and after.

    Parameters
    ----------
    contents: str
        The string containing an XML document
    root_tag: str
        The root tag that should be in the document

    Returns
    -------
    ET.Element
        The XML Element starting with the root tag
    """
    tag_to_use = root_tag
    if root_tag.startswith("<"):
        tag = re.search(r"<([^\s]*).*>", root_tag)
        if tag:
            tag_to_use = tag
        else:
            raise ValueError(f"Invalid root tag: {root_tag}")

    begin_tag_starts_at = contents.find(f"<{tag_to_use}")
    if begin_tag_starts_at == -1:
        raise ValueError(f"<{tag_to_use}> does not seem to appear in the document")
    contents = contents[begin_tag_starts_at:]

    end_tag = f"</{tag_to_use}>"
    end_tag_starts_at = contents.rfind(end_tag)
    end_tag_ends_at = end_tag_starts_at + len(tag_to_use) + 3
    contents = contents[:end_tag_ends_at]
    return ET.fromstring(contents)
