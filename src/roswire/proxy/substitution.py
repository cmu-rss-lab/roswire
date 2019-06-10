# -*- coding: utf-8 -*-
"""
This module provides handling of XML substitution arguments, which are used
by XML launch and xacro files.
"""
__all__ = ('resolve',)

import re
import functools
import logging

from .shell import ShellProxy
from .file import FileProxy

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

R_ARG = re.compile(r'(?<=\$\().+?(?=\))')


def resolve_arg(shell: ShellProxy,
                files: FileProxy,
                s: str
                ) -> str:
    logger.debug("resolving substitution argument: %s", s)
    kind, *params = s.split(' ')
    logger.debug("argument kind: %s", kind)
    return s


def resolve(shell: ShellProxy,
            files: FileProxy,
            s: str
            ) -> str:
    r = functools.partial(resolve_arg, shell, files)
    return R_ARG.sub(lambda m: r(m.group(0)), s)
