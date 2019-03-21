# -*- coding: utf-8 -*-
"""
This module implements a proxy for interacting with gcov within a Docker
container.
"""
__all__ = ('GcovProxy',)

from .shell import ShellProxy


class GcovProxy:
    """
    Provides an interface to gcov within a Docker container.

    This proxy can be used to obtain coverage reports for C and C++ programs.
    """
    def __init__(self, shell: ShellProxy) -> None:
        self.__shell = shell
