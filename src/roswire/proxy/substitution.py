# -*- coding: utf-8 -*-
"""
This module provides handling of XML substitution arguments, which are used
by XML launch and xacro files.
"""
__all__ = ('resolve',)

from typing import Optional, Dict, Any
import os
import re
import shlex
import functools
import logging

from .shell import ShellProxy
from .file import FileProxy
from ..exceptions import EnvNotFoundError, SubstitutionError

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

R_ARG = re.compile(r'\$\(.+?\)')


def resolve_arg(shell: ShellProxy,
                files: FileProxy,
                s: str,
                context: Optional[Dict[str, Any]] = None
                ) -> str:
    """
    Raises
    ------
    EnvNotFoundError
        if a given environment variable is not found.
    """
    if not context:
        context = {}

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
    if kind == 'dirname':
        try:
            dirname = os.path.dirname(context['filename'])
        except KeyError:
            m = 'filename is not provided by the launch context'
            raise SubstitutionError(m)
        dirname = os.path.normpath(dirname)
        return dirname
    if kind == 'arg':
        arg_name = params[0]
        if 'arg' not in context or arg_name not in context['arg']:
            m = f'arg not supplied to launch context [{arg_name}]'
            raise SubstitutionError(m)
        return context['arg'][arg_name]
    if kind == 'find':
        package = params[0]
        cmd = f'rospack find {shlex.quote(package)}'
        retcode, location, duration = shell.execute(cmd)
        if retcode != 0:
            raise SubstitutionError(f'failed to locate package: {package}')
        return location.strip()

    # TODO $(anon name)
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
