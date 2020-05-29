# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchController',)

import attr
import dockerblade


@attr.s(frozen=True, slots=True, auto_attribs=True)
class ROSLaunchController:
    """Provides an interface to a roslaunch process.

    Attributes
    ----------
    filename: str
        The absolute path of the XML launch file used by this process.
    """
    filename: str
    _popen: dockerblade.popen.Popen = attr.ib(repr=False)

    def terminate(self) -> None:
        """Terminates this roslaunch process."""
        self._popen.terminate()
