# -*- coding: utf-8 -*-
"""
This module provides a bag file writer that writes the contents of a bag to a
binary file on disk.

See
---
https://github.com/ros/ros_comm/blob/melodic-devel/tools/rosbag/src/rosbag/bag.py
"""
__all__ = ('BagWriter',)

from typing import BinaryIO, Iterable, Dict, Type, Tuple

from .core import (BagMessage, OpCode, Compression, ConnectionInfo, Chunk,
                   Index, IndexEntry, ChunkConnection)
from ..definitions import Message
from ..definitions.encode import *

BIN_CHUNK_INFO_VERSION = encode_uint32(1)
BIN_INDEX_VERSION = encode_uint32(1)


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
        self.__connections: Dict[str, ConnectionInfo] = {}
        self.__chunks: List[Chunk] = []
        self.__pos_header = 0
        self.__pos_chunks = 0
        self.__pos_index = 0

    @property
    def filename(self) -> str:
        return self.__fn

    def _write_header(self,
                      code: Optional[OpCode],
                      fields: Dict[str, bytes]
                      ) -> None:
        # write a length of zero for now and correct that length once the
        # fields have been written
        pos_size = self.__fp.tell()
        write_uint32(0, self.__fp)
        pos_content = self.__fp.tell()

        if code:
            fields['op'] = code.value
        for name, bin_value in fields.items():
            bin_name = f'{name}='.encode('utf-8')
            size_field = len(bin_name) + len(bin_value)
            write_uint32(size_field, self.__fp)
            self.__fp.write(bin_name)
            self.__fp.write(bin_value)

        # correct the length
        pos_end = self.__fp.tell()
        size_total = pos_end - pos_content
        self.__fp.seek(pos_size)
        write_uint32(size_total, self.__fp)
        self.__fp.seek(pos_end)

    def _write_header_record(self) -> None:
        self.__fp.seek(self.__pos_header)
        self._write_header(OpCode.HEADER, {
            'index_pos': encode_uint64(self.__pos_index),
            'conn_count': encode_uint32(len(self.__connections)),
            'chunk_count': encode_uint32(len(self.__chunks))})

        # ensure the bag header record is 4096 characters long by padding it
        # with ASCII space characters (0x20) where necessary.
        pos_current = self.__fp.tell()
        size = 4096
        size_header = pos_current - self.__pos_header
        size_padding = size - size_header - 4

        write_uint32(size_padding, self.__fp)
        padding = b'\x20' * size_padding
        self.__fp.write(padding)

    def _write_message(self,
                       offset: int,
                       index: Index,
                       message: BagMessage,
                       ) -> None:
        typ = message.message.__class__
        connection = self._get_connection(message.topic, typ)

        pos_header = self.__fp.tell()
        self._write_header(OpCode.MESSAGE_DATA, {
            'conn': encode_uint32(connection.conn),
            'time': encode_time(message.time)})

        bin_data = message.message.encode()
        size_data = len(bin_data)
        write_uint32(size_data, self.__fp)
        self.__fp.write(bin_data)

        # update index
        index_entry = IndexEntry(time=message.time,
                                 pos=pos_header,
                                 offset=offset)
        if connection.conn not in index:
            index[connection.conn] = []
        index[connection.conn].append(index_entry)

    def _write_chunk_data(self, messages: Iterable[BagMessage]) -> Index:
        index: Index = {}
        pos_start = self.__fp.tell()
        pos_end = pos_start
        offset = 0
        for m in messages:
            self._write_message(offset, index, m)
            pos_end = self.__fp.tell()
            size_record = pos_end - pos_start
            offset += size_record
            pos_start = pos_end
        return index

    def _write_chunk_record(self,
                            compression: Compression,
                            messages: Iterable[BagMessage]
                            ) -> Tuple[Chunk, Index]:
        bin_compression = compression.value.encode('utf-8')

        # for now, we write a bogus header and size field
        # once we've finished writing the data, we'll correct them
        pos_header = self.__fp.tell()
        self._write_header(OpCode.CHUNK, {
            'compression': bin_compression,
            'size': encode_uint32(0)})
        write_uint32(0, self.__fp)
        pos_data = self.__fp.tell()

        # write chunk contents
        index = self._write_chunk_data(messages)

        # determine time of earliest and latest message in the bag
        time_start = time_end = Time(0, 0)
        for time in (e.time for ci in index.values() for e in ci):
            time_start = min(time, time_start)
            time_end = max(time, time_end)

        # compute chunk size
        pos_end = self.__fp.tell()
        size_compressed = pos_end - pos_data
        size_uncompressed = size_compressed

        # update header and size
        self.__fp.seek(pos_header)
        self._write_header(OpCode.CHUNK, {
            'compression': bin_compression,
            'size': encode_uint32(size_uncompressed)})
        write_uint32(size_compressed, self.__fp)
        self.__fp.seek(pos_end)

        # build a description of the chunk
        conns = [ChunkConnection(conn, len(entries))
                 for conn, entries in index.items()]
        chunk = Chunk(pos_record=pos_header,  # type: ignore
                      pos_data=pos_data,
                      time_start=time_start,
                      time_end=time_end,
                      connections=conns,
                      compression=compression,
                      size_compressed=size_compressed,
                      size_uncompressed=size_uncompressed)
        return chunk, index

    def _write_connection_index(self,
                                conn: int,
                                entries: List[IndexEntry]
                                ) -> None:
        num_entries = len(entries)
        size_data = num_entries * 12
        self._write_header(OpCode.INDEX_DATA, {
            'ver': BIN_INDEX_VERSION,
            'conn': encode_uint32(conn),
            'count': encode_uint32(num_entries)})
        write_uint32(size_data, self.__fp)
        for entry in entries:
            write_time(entry.time, self.__fp)
            write_uint32(entry.offset, self.__fp)

    def _write_chunk(self,
                     compression: Compression,
                     messages: Iterable[BagMessage]
                     ) -> None:
        # TODO for now, we only support uncompressed writing
        assert compression == Compression.NONE
        chunk, index = self._write_chunk_record(compression, messages)
        self.__chunks.append(chunk)
        for conn, entries in index.items():
            self._write_connection_index(conn, entries)

    def _write_connection_record(self, conn: ConnectionInfo) -> None:
        self._write_header(OpCode.CONNECTION_INFO, {
            'conn': encode_uint32(conn.conn),
            'topic': conn.topic.encode('utf-8')})
        pos_size = self.__fp.tell()
        write_uint32(0, self.__fp)

        # write the connection header
        header_conn: Dict[str, bytes] = {}
        header_conn['topic'] = conn.topic_original.encode('utf-8')
        header_conn['type'] = conn.typ.encode('utf-8')
        header_conn['md5sum'] = conn.md5sum.encode('utf-8')
        header_conn['message_definition'] = \
            conn.message_definition.encode('utf-8')
        if conn.callerid is not None:
            header_conn['callerid'] = conn.callerid.encode('utf-8')
        if conn.latching is not None:
            header_conn['latching'] = conn.latching.encode('utf-8')
        self._write_header(None, header_conn)

        # update the record size
        pos_end = self.__fp.tell()
        size_data = pos_end - pos_size
        self.__fp.seek(pos_size)
        write_uint32(size_data, self.__fp)
        self.__fp.seek(pos_end)

    def _get_connection(self,
                        topic: str,
                        typ: Type[Message]
                        ) -> ConnectionInfo:
        # if there isn't a connection for the topic, create one and write a
        # connection record.
        if topic not in self.__connections:
            msg_format = typ.format
            conn = len(self.__connections)
            info = ConnectionInfo(conn=conn,
                                  topic=topic,
                                  topic_original=topic,
                                  typ=msg_format.fullname,
                                  md5sum=typ.md5sum(),
                                  message_definition=msg_format.definition,
                                  callerid=None,
                                  latching=None)
            self.__connections[topic] = info
            self._write_connection_record(info)

        return self.__connections[topic]

    def _write_chunk_info_record(self, chunk: Chunk) -> None:
        num_connections = len(chunk.connections)
        pos_header = self.__fp.tell()
        self._write_header(OpCode.CHUNK_INFO, {
            'ver': BIN_CHUNK_INFO_VERSION,
            'chunk_pos': encode_uint64(chunk.pos_record),
            'start_time': encode_time(chunk.time_start),
            'end_time': encode_time(chunk.time_end),
            'count': encode_uint32(num_connections)})

        size_data = num_connections * 8
        write_uint32(size_data, self.__fp)
        for connection in chunk.connections:
            write_uint32(connection.uid, self.__fp)
            write_uint32(connection.count, self.__fp)

    def _write_index(self) -> None:
        for connection in self.__connections.values():
            self._write_connection_record(connection)
        for chunk in self.__chunks:
            self._write_chunk_info_record(chunk)

    def write(self, messages: Iterable[BagMessage]) -> None:
        """
        Writes a sequence of messages to the bag.
        Any existing bag file contents will be overwritten.
        """
        self.__fp.truncate(0)
        self.__fp.write('#ROSBAG V2.0\n'.encode('utf-8'))

        # create a placeholder header for now
        self.__pos_header = self.__fp.tell()
        self._write_header_record()

        # for now, we write to a single, uncompressed chunk
        # each chunk record is followed by a sequence of IndexData record
        # - each connection in the chunk is represented by an IndexData record
        self.__pos_chunks = self.__fp.tell()
        self._write_chunk(Compression.NONE, messages)

        # write index
        self.__pos_index = self.__fp.tell()
        self._write_index()

        # fix the header
        self._write_header_record()

    def close(self) -> None:
        self.__fp.close()
