# -*- coding: utf-8 -*-
__all__ = ('BagReader',)

import heapq
import os
from functools import reduce
from io import BytesIO
from typing import (BinaryIO, Collection, Dict, Iterator, List, Optional,
                    Set, Tuple, Type)

from loguru import logger

from .core import (BagHeader, BagMessage, Chunk, ChunkConnection, Compression,
                   ConnectionInfo, Index, IndexEntry, OpCode)
from ...common.base import Duration, Time
from ...common.decode import (decode_string,
                              decode_time,
                              decode_uint32,
                              decode_uint64,
                              read_encoded_header,
                              read_sized,
                              read_time,
                              read_uint32)
from ...common.msg import Message
from ...common.type_db import TypeDatabase


class BagReader:
    def __init__(self, fn: str, db_type: TypeDatabase) -> None:
        self.__fp = open(fn, 'rb')
        self.__db_type = db_type
        self.__size_bytes: int = os.path.getsize(fn)

        version = self._read_version()
        assert version == '#ROSBAG V2.0'
        logger.debug(f"bag version: {version}")

        self.__header = self._read_header_record()
        logger.debug(f"bag header: {self.__header}")

        # skip past chunks
        self._seek(self.__header.index_pos)

        # obtain a list of all connections in the bag
        connections: List[ConnectionInfo] = []
        for _ in range(self.__header.conn_count):
            conn = self._read_connection_record()
            connections.append(conn)
        self.__connections: Tuple[ConnectionInfo, ...] = tuple(connections)

        # obtain a summary of each chunk
        chunks: List[Chunk] = []
        for _ in range(self.__header.chunk_count):
            info = self._read_chunk_info_record()
            chunks.append(info)
        self.__chunks: Tuple[Chunk, ...] = tuple(chunks)
        self.__pos_to_chunk: Dict[int, Chunk] = \
            {c.pos_record: c for c in chunks}

        # read the index
        self.__index: Index = self._read_index()
        logger.debug(f"topics: {self.topics}")
        for conn_id, indices in self.__index.items():
            topic = self.__connections[conn_id].topic
            logger.debug(f"conn {conn_id} [{topic}]: {len(indices)} messages")

    @property
    def connections(self) -> Tuple[ConnectionInfo, ...]:
        return self.__connections

    @property
    def header(self) -> BagHeader:
        return self.__header

    @property
    def chunks(self) -> Tuple[Chunk, ...]:
        return self.__chunks

    @property
    def index(self) -> Index:
        return self.__index

    @property
    def duration(self) -> Duration:
        """The duration of the recording in this bag file."""
        return Duration.between(self.time_start, self.time_end)

    @property
    def time_start(self) -> Time:
        """The time at which the recording began."""
        times = [c.time_start for c in self.chunks]
        return reduce(min, times[1:], times[0])

    @property
    def time_end(self) -> Time:
        """The time at which the recording ended."""
        times = [c.time_end for c in self.chunks]
        return reduce(max, times[1:], times[0])

    @property
    def topics(self) -> Set[str]:
        """The names of all topics represented in this bag."""
        return set(c.topic for c in self.connections)

    @property
    def topics_to_types(self) -> Dict[str, Type[Message]]:
        """A mapping from topics to their respective types."""
        return {c.topic: self.__db_type[c.typ] for c in self.connections}

    def _seek(self, pos: int, ptr: Optional[BinaryIO] = None) -> None:
        ptr = ptr if ptr else self.__fp
        ptr.seek(pos)

    def _skip_sized(self, ptr: Optional[BinaryIO] = None) -> None:
        ptr = ptr if ptr else self.__fp
        ptr.seek(read_uint32(ptr), os.SEEK_CUR)

    def _skip_record(self, ptr: Optional[BinaryIO] = None) -> None:
        self._skip_sized(ptr)
        self._skip_sized(ptr)

    def _read_version(self) -> str:
        return decode_string(self.__fp.readline()).rstrip()

    def _read_header(self,
                     op_expected: Optional[OpCode] = None,
                     *,
                     ptr: Optional[BinaryIO] = None
                     ) -> Dict[str, bytes]:
        fields = read_encoded_header(ptr if ptr else self.__fp)
        if op_expected:
            assert 'op' in fields
            op_actual: OpCode = OpCode(fields['op'])
            if fields['op'] != op_expected.value:
                m = ("unexpected opcode when reading header: "
                     f"expected {op_expected.name} [{op_expected.hex}] "
                     f"but was {op_actual.name} [{op_actual.hex}].")
                raise Exception(m)
        return fields

    def _read_header_record(self) -> BagHeader:
        header = self._read_header(OpCode.HEADER)
        index_pos = decode_uint64(header['index_pos'])
        conn_count = decode_uint32(header['conn_count'])
        chunk_count = decode_uint32(header['chunk_count'])
        self._skip_sized()
        return BagHeader(index_pos, conn_count, chunk_count)

    def _read_connection_record(self) -> ConnectionInfo:
        header = self._read_header(OpCode.CONNECTION_INFO)
        conn = self._read_header()
        logger.debug(f"conn record header: {header}")
        logger.debug(f"conn header: {conn}")
        callerid: Optional[str] = None
        latching: Optional[str] = None
        if 'callerid' in conn:
            callerid = decode_string(conn['callerid'])
        if 'latching' in conn:
            latching = decode_string(conn['latching'])
        return ConnectionInfo(conn=decode_uint32(header['conn']),
                              callerid=callerid,
                              latching=latching,
                              topic=decode_string(header['topic']),
                              topic_original=decode_string(conn['topic']),
                              typ=decode_string(conn['type']),
                              md5sum=decode_string(conn['md5sum']),
                              message_definition=decode_string(conn['message_definition']))  # noqa

    def _read_chunk_info_record(self) -> Chunk:
        header = self._read_header(OpCode.CHUNK_INFO)
        ver: int = decode_uint32(header['ver'])
        assert ver == 1
        pos_record: int = decode_uint64(header['chunk_pos'])
        time_start: Time = decode_time(header['start_time'])
        time_end: Time = decode_time(header['end_time'])
        num_connections: int = decode_uint32(header['count'])

        # obtain a summary of the number of messages for each connection
        # represented in this chunk
        connections: List[ChunkConnection] = []
        contents: bytes = read_sized(self.__fp)
        for _ in range(num_connections):
            uid = decode_uint32(contents[0:4])
            count = decode_uint32(contents[4:8])
            connections.append(ChunkConnection(uid, count))
            contents = contents[8:]

        # read connection index records?
        assert not contents

        # read the chunk header
        pos_original = self.__fp.tell()
        self.__fp.seek(pos_record)
        header = self._read_header(OpCode.CHUNK)
        size_uncompressed = decode_uint32(header['size'])
        compression = Compression(decode_string(header['compression']))
        pos_data = self.__fp.tell()

        # determine the compressed size of the chunk data
        size_compressed = read_uint32(self.__fp)

        # restore the original position of the read pointer
        self.__fp.seek(pos_original)

        chunk = Chunk(pos_record=pos_record,
                      pos_data=pos_data,
                      time_start=time_start,
                      time_end=time_end,
                      size_uncompressed=size_uncompressed,
                      size_compressed=size_compressed,
                      compression=compression,
                      connections=connections)

        logger.debug("decoded chunk: {chunk}")
        return chunk

    def _read_index(self) -> Index:
        logger.debug("reading index")
        index: Index = {c.conn: [] for c in self.connections}
        for chunk in self.chunks:
            logger.debug(f"reading index for chunk: {chunk}")
            pos = chunk.pos_record
            self._seek(pos)
            self._skip_record()
            for _ in range(len(chunk.connections)):
                self._read_index_record(pos, index)
        logger.debug("read index")

        logger.debug("pruning empty connections from index")
        for c in [cc for cc in index if not index[cc]]:
            del index[c]
            # TODO delete connection!
        logger.debug("pruned index")
        return index

    def _read_index_record(self, pos_chunk: int, index: Index) -> None:
        header = self._read_header(OpCode.INDEX_DATA)
        ver = decode_uint32(header['ver'])
        uid = decode_uint32(header['conn'])
        count = decode_uint32(header['count'])
        assert ver == 1

        read_uint32(self.__fp)  # skip size
        for _ in range(count):
            time = read_time(self.__fp)
            offset = read_uint32(self.__fp)
            entry = IndexEntry(time=time, pos=pos_chunk, offset=offset)
            index[uid].append(entry)

    def _get_connections(self,
                         topics: Optional[Collection[str]] = None
                         ) -> Iterator[ConnectionInfo]:
        """
        Returns an iterator over all connections for a given set of topics.
        """
        if not topics:
            yield from self.connections
        else:
            yield from (c for c in self.connections if c.topic in topics)

    def _get_entries(self,
                     connections: Collection[ConnectionInfo],
                     time_start: Optional[Time] = None,
                     time_end: Optional[Time] = None
                     ) -> Iterator[IndexEntry]:
        entries = heapq.merge(*[self.__index[c.conn] for c in connections])
        for entry in entries:
            if time_start and entry.time < time_start:
                continue
            if time_end and entry.time > time_end:
                return
            yield entry

    def fetch_message_data_record(self, pos: int, offset: int) -> BagMessage:
        # find the chunk to which the message belongs
        # read the contents of that chunk to a bytes buffer
        chunk = self.__pos_to_chunk[pos]
        self._seek(chunk.pos_data)
        if chunk.compression == Compression.NONE:
            bfr = BytesIO(read_sized(self.__fp))
        else:
            raise NotImplementedError

        # seek position of message data record
        # - skip any preceding connection records
        self._seek(offset, bfr)
        while True:
            header = self._read_header(ptr=bfr)
            op = OpCode(header['op'])
            if op == OpCode.CONNECTION_INFO:
                self._skip_sized(bfr)
                continue
            if op == OpCode.MESSAGE_DATA:
                conn_id = decode_uint32(header['conn'])
                t = decode_time(header['time'])
                break
            m = "unexpected opcode: got {} but expected {}"
            m = m.format(op, OpCode.MESSAGE_DATA)
            raise Exception(m)

        # fetch the topic name and message type
        conn_info = self.__connections[conn_id]
        topic = conn_info.topic
        msg_typ_name = conn_info.typ
        msg_typ = self.__db_type[msg_typ_name]

        # read the raw message data
        raw = read_sized(bfr)
        content = msg_typ.read(BytesIO(raw))
        msg = BagMessage(topic, t, content)
        logger.debug(f"decoded message: {msg}")
        return msg

    def read_messages(self,
                      topics: Optional[Collection[str]] = None,
                      time_start: Optional[Time] = None,
                      time_end: Optional[Time] = None
                      ) -> Iterator[BagMessage]:
        conns = set(self._get_connections(topics))
        for entry in self._get_entries(conns, time_start, time_end):
            yield self.fetch_message_data_record(entry.pos, entry.offset)

    def __iter__(self) -> Iterator[BagMessage]:
        yield from self.read_messages()
