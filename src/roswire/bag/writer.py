# -*- coding: utf-8 -*-
__all__ = ('BagWriter',)

from typing import BinaryIO, Iterable

from .core import BagMessage, OpCode
from ..definitions.encode import *


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

    def _write_header_record(self,
                             pos_index: int,
                             conn_count: int,
                             chunk_count: int
                             ) -> None:
        self.__fp.seek(self.__pos_header)
        self._write_record_header(OpCode.HEADER,
            {'index_pos': encode_uint64(pos_index),
             'conn_count': encode_uint32(conn_count),
             'chunk_count': encode_uint32(chunk_count)})

        # ensure the bag header record is 4096 characters long by padding it
        # with ASCII space characters (0x20) where necessary.
        pos_current = self.__fp.tell()
        size = 4096
        size_header = pos_current - self.__pos_header
        size_padding = size - size_header - 4

        write_uint32(size_padding, self.__fp)
        padding = b'\x20' * size_padding
        self.__fp.write(padding)

    def write(self, messages: Iterable[BagMessage]) -> None:
        """
        Writes a sequence of messages to the bag.
        Any existing bag file contents will be overwritten.
        """
        self.__fp.write('#ROSBAG V2.0\n'.encode('utf-8'))

        # create a placeholder header for now
        self.__pos_header = self.__fp.tell()
        self._write_header_record(0, 0, 0)

        # write chunks
        raise NotImplementedError

        # write index
