# -*- coding: utf-8 -*-
__all__ = ('BagWriter',)

from .core import BagMessage


class BagWriter:
    """
    Provides an interface for writing messages to a bag file on disk.

    Attributes
    ----------
    filename: str
        The name of the file to which the bag will be written.
    """
    def __init__(self, fn: str) -> None:
        self.__fn = fn

    @property
    def filename(self) -> str:
        return self.__fn
