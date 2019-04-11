# -*- coding: utf-8 -*-
__all__ = ('BagWriter',)

from typing import BinaryIO

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
        self.__fp: BinaryIO = open(fn, 'wb')

    @property
    def filename(self) -> str:
        return self.__fn

    def write(self, messages: Iterable[BagMessage]) -> None:
        """
        Writes a sequence of messages to the bag.
        Any existing bag file contents will be overwritten.
        """
        raise NotImplementedError
