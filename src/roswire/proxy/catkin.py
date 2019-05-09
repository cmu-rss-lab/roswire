# -*- coding: utf-8 -*-
# http://wiki.ros.org/Bags/Format/2.0
__all__ = ('CatkinProxy',)

from typing import Optional, List

from shell import ShellProxy


class CatkinProxy:
    def __init__(self, shell: ShellProxy) -> None:
        self._shell = shell

    def build(self,
              packages: Optional[List[str]] = None,
              no_deps: bool = False,
              pre_clean: bool = False,
              jobs: Optional[int] = None
              ) -> None:
        pass
