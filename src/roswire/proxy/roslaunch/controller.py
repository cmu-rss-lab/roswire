# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchController',)

from typing import Optional

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
    pid: Optional[int]
        The PID of the launch process inside the container, if known.
    """
    filename: str
    _popen: dockerblade.popen.Popen = attr.ib(repr=False)

    @property
    def pid(self) -> Optional[int]:
        return self._popen.pid

    @property
    def command(self) -> str:
        return self._popen.args

    def terminate(self) -> None:
        """Terminates this roslaunch process."""
        self._popen.terminate()

    def close(self) -> None:
        """Terminates this roslaunch process."""
        self.terminate()
