# -*- coding: utf-8 -*-
# http://wiki.ros.org/Bags/Format/2.0
__all__ = ('CatkinProxy',)

from shell import ShellProxy


class CatkinProxy:
    def __init__(self, shell: ShellProxy) -> None:
        self._shell = shell
