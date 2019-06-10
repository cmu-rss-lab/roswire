# -*- coding: utf-8 -*-
__all__ = ('resolve',)

from .shell import ShellProxy
from .files import FileProxy


def resolve(shell: ShellProxy,
            files: FileProxy,
            s: str
            ) -> str:
    return
