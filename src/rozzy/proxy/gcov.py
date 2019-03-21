# -*- coding: utf-8 -*-
"""
This module implements a proxy for interacting with gcov within a Docker
container.
"""
__all__ = ('GcovProxy',)

from typing import Dict, Set
import shlex
import logging
import xml.etree.ElementTree as ET

from bugzoo.core.fileline import FileLineSet

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

    @staticmethod
    def _parse_report_string(s: str) -> FileLineSet:
        """Obtains the set of executed lines from a gcovr XML report.

        Parameters
        ----------
        s: str
            the contents of the XML report.

        Returns
        -------
        bugzoo.core.FileLineSet
            the set of executed lines
        """
        files_to_lines: Dict[str, Set[int]] = {}

        root = ET.fromstring(s)
        pkgs = root.find('packages').findall('package')
        classes = [c for p in pkgs for c in p.find('classes').findall('class')]
        fn_to_report = {c.attrib['filename']: c.find('lines').findall('line')
                        for c in classes}
        fn_to_executed = {
            fn: set(int(l.attrib['number']) for l in lines
                    if int(l.attrib['hits']) > 0)}

        # TODO remove instrumentation if directed to do so
        return FileLineSet(fn_to_executed)

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
            return self._parse_report_string(str_report)
