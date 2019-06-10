# -*- coding: utf-8 -*-
"""
This module provides handling of XML substitution arguments, which are used
by XML launch and xacro files.
"""
__all__ = ('resolve',)

from typing import Optional, Dict
import re
import functools
import logging

from .shell import ShellProxy
from .file import FileProxy
from ..exceptions import EnvNotFoundError

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

R_ARG = re.compile(r'\$\(.+?\)')


def resolve_arg(shell: ShellProxy,
                files: FileProxy,
                s: str,
                context: Optional[Dict[str, str]] = None
                ) -> str:
    """
    Raises
    ------
    EnvNotFoundError
        if a given environment variable is not found.
    """
    logger.debug("resolving substitution argument: %s", s)
    s = s[2:-1]
    logger.debug("stripped delimiters: %s", s)
    kind, *params = s.split(' ')
    logger.debug("argument kind: %s", kind)

    if kind == 'env':
        return shell.environ(params[0])
    if kind == 'optenv':
        try:
            return shell.environ(params[0])
        except EnvNotFoundError:
            return ' '.join(params[1:])

    # TODO $(arg foo)
    # TODO $(find pkg)
    # TODO $(anon name)
    # TODO $(dirname)
    return s


def resolve(shell: ShellProxy,
            files: FileProxy,
            s: str,
            context: Optional[Dict[str, str]] = None
            ) -> str:
    # TODO $(eval ...)
    if s.startswith('$(eval ') and s[-1] == ')':
        raise NotImplementedError
    r = functools.partial(resolve_arg, shell, files)
    return R_ARG.sub(lambda m: r(m.group(0), context), s)
