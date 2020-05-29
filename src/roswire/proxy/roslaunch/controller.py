# -*- coding: utf-8 -*-
__all__ = ('ROSLaunchController',)

import attr
import dockerblade


@attr.s(frozen=True, slots=True, auto_attribs=True)
class ROSLaunchController:
    _popen: dockerblade.popen.Popen

    def terminate(self) -> None:
        """Terminates this roslaunch process."""
        self._popen.terminate()
