# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchController',)

from types import TracebackType
from typing import Iterator, Optional, Type

import attr
import dockerblade


@attr.s(frozen=True, slots=True, auto_attribs=True)
class ROSLaunchController:
    """Provides an interface to a roslaunch process.

    Attributes
    ----------
    filename: str
        The absolute path of the XML launch file used by this process.
    command: str
        The command string that was used by this process.
    popen: dockerblade.popen.Popen
        An interface to the underlying exec process for this roslaunch process.
    pid: Optional[int]
        The PID of the launch process inside the container, if known.
    """
    filename: str
    popen: dockerblade.popen.Popen = attr.ib(repr=False)

    def __enter__(self) -> 'ROSLaunchController':
        return self

    def __exit__(self,
                 ex_type: Optional[Type[BaseException]],
                 ex_val: Optional[BaseException],
                 ex_tb: Optional[TracebackType]
                 ) -> None:
        self.close()

    @property
    def stream(self) -> Iterator[str]:
        yield from self.popen.stream()  # type: ignore

    def is_running(self) -> bool:
        """Checks whether or not this roslaunch process is still running."""
        return not self.popen.finished

    def terminate(self) -> None:
        """Terminates this roslaunch process."""
        self.popen.terminate()

    def close(self) -> None:
        """Terminates this roslaunch process."""
        self.terminate()
