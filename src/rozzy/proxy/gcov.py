# -*- coding: utf-8 -*-
"""
This module implements a proxy for interacting with gcov within a Docker
container.
"""
__all__ = ('GcovProxy',)

import shlex
import logging

from .shell import ShellProxy
from .file import FileProxy
from ..exceptions import GcovException

logging: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class GcovProxy:
    """
    Provides an interface to gcov within a Docker container.

    This proxy can be used to obtain coverage reports for C and C++ programs.

    Note
    ----
        For now, this module actually provides an interface to gcovr rather
        than gcov. In the future, this module will interact with gcov
        directly, avoiding the need to install gcovr inside the Docker image
        for the ROS application.
    """
    def __init__(self, shell: ShellProxy, files: FileProxy) -> None:
        self.__shell = shell
        self.__files = files

    def extract(self,
                dir_src: str,
                *,
                delete: bool = False
                ) -> None:
        with self.__files.tempfile() as fn_temp:
            cmd = (f'gcovr -x '
                   f'-o {shlex.quote(fn_temp)} '
                   f"{('-d ' if delete else '')}"
                   f'-r {shlex.quote(dir_src)}')

            code, output, duration = self.__shell.execute(cmd)
            if code != 0:
                m = f"gcovr produced non-zero return code [{code}]"
                raise GcovException(m)
            logger.debug('took %.2f seconds to run gcovr', duration)

            str_report = self.__files.read(fn_temp)
            return self.__parse_report_string(str_report)
