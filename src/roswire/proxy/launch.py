# -*- coding: utf-8 -*-
"""
This file implements a proxy for parsing the contents of launch files.
"""
__all__ = ('LaunchFileReader',)

import logging

import lxml

from .file import FileProxy

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class LaunchFileReader:
    def __init__(self, files: FileProxy) -> None:
        self.__files = files
