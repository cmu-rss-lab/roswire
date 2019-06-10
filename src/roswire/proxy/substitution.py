# -*- coding: utf-8 -*-
"""
This module provides handling of XML substitution arguments, which are used
by XML launch and xacro files.
"""
__all__ = ('resolve',)

from .shell import ShellProxy
from .files import FileProxy


def resolve(shell: ShellProxy,
            files: FileProxy,
            s: str
            ) -> str:
    return
