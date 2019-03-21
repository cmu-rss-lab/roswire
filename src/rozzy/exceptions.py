# -*- coding: utf-8 -*-
"""
This module contains all of Rozzy's exceptions.
"""


class RozzyException(Exception):
    """Base class used by all Rozzy exceptions."""


class RecorderAlreadyStarted(RozzyException):
    """Recording has already started."""


class RecorderNotStarted(RozzyException):
    """Recording has not begun."""


class RecorderAlreadyStopped(RozzyException):
    """Recording has already stopped."""


class ParsingError(RozzyException):
    """Rozzy failed to parse a given file/string."""


class NodeNotFoundError(KeyError, RozzyException):
    """
    No node was found with the given name.
    """
    def __init__(self, name: str) -> None:
        super().__init__(f"node not found: {name}")


class ServiceNotFoundError(KeyError, RozzyException):
    """No service was found with the given name."""
    def __init__(self, name: str) -> None:
        super().__init__(f"service not found: {name}")


class ParameterNotFoundError(KeyError, RozzyException):
    """No parameter was found with the given name."""
    def __init__(self, name: str) -> None:
        super().__init__(f"parameter not found: {name}")


class GcovException(RozzyException):
    """An error occurred when running gcov."""
