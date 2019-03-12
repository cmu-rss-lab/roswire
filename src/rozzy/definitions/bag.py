"""
Reference: https://github.com/strawlab/ros_comm/blob/master/tools/rosbag/src/rosbag/bag.py
"""
__all__ = ['BagReader']

from typing import Dict, Sequence, Union, Optional, Tuple, List, Type
from io import BytesIO
from enum import Enum
import os
import bz2
import struct
import logging

import attr

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def decode_uint32(v: bytes) -> int: return struct.unpack('<L', v)[0]
def decode_int(v: bytes) -> int: return int.from_bytes(v, 'little')
def decode_str(v: bytes) -> str: return v.decode('utf-8')


class OpCode(Enum):
    MESSAGE_DATA = b'\x02'
    HEADER = b'\x03'
    INDEX_DATA = b'\x04'
    CHUNK = b'\x05'
    CHUNK_INFO = b'\x06'
    CONNECTION_INFO = b'\x07'

    @property
    def hex(self) -> str:
        return f'0x{self.value.hex()}'


@attr.s(frozen=True)
class ChunkRecord:
    compression: str = attr.ib()
    size: int = attr.ib()

#    @classmethod
#    def from_stream_with_header(cls,
#                                s: BytesIO,
#                                header: RecordHeader
#                                ) -> 'ChunkRecord':
#        assert header['op'] == b'\x05'
#        compression: str = header['compression'].decode('utf-8')
#        size_uncompressed = int.from_bytes(header['size'], 'little')
#        assert compression in ['none', 'bz2']
#
#        size_compressed = int.from_bytes(s.read(4), 'little')
#        data_bytes = s.read(size_compressed)
#        if compression == 'bz2':
#            data_bytes = bz2.decompress(data_bytes)
#        s = BytesIO(data_bytes)
#
#        records: List[Union[MessageDataRecord, ConnectionRecord]] = []
#        while s.tell() < size_uncompressed:
#            record = BagRecord.from_stream(s)
#            records.append(record)
#
#        return ChunkRecord(compression, size_uncompressed, records)


@attr.s(frozen=True)
class ConnectionInfo:
    conn: int = attr.ib()
    topic: str = attr.ib()
    topic_original: str = attr.ib()
    typ: str = attr.ib()
    md5sum: str = attr.ib()
    message_definition: str = attr.ib()
    callerid: Optional[str] = attr.ib()
    latching: Optional[str] = attr.ib()


@attr.s(frozen=True)
class BagHeader:
    index_pos: int = attr.ib()
    conn_count: int = attr.ib()
    chunk_count: int = attr.ib()


class BagReader:
    def __init__(self, fn: str) -> None:
        self.__fp = open(fn, 'rb')
        self.__size_bytes: int = os.path.getsize(fn)

        version = self._read_version()
        assert version == '#ROSBAG V2.0'
        logger.debug("bag version: %s", version)

        self.__header = self._read_header_record()
        logger.debug("bag header: %s", self.__header)

        # read connection records
        self._seek(self.__header.index_pos)
        connections: List[ConnectionRecord] = []
        for i in range(conn_count):
            conn = self._read_connection_record()
            connections.append(conn)
            print(conn)

    def _seek(self, pos: int) -> None:
        self.__fp.seek(pos)

    def _skip_sized(self) -> None:
        self.__fp.seek(self._read_uint32(), os.SEEK_CUR)

    def _read_sized(self) -> bytes:
        size = self._read_uint32()
        logger.debug("reading sized block: %d bytes", size)
        return self.__fp.read(size)

    def _read_uint32(self) -> int:
        return decode_uint32(self.__fp.read(4))

    def _read_version(self) -> str:
        return decode_str(self.__fp.readline()).rstrip()

    def _read_header(self,
                     op_expected: Optional[OpCode] = None
                     ) -> Dict[str, bytes]:
        fields: Dict[str, bytes] = {}
        header = self._read_sized()
        while header:
            size = decode_int(header[:4])
            header = header[4:]
            name, sep, value = header[:size].partition(b'\x3d')
            if sep == '':
                raise Exception('error reading header field')
            fields[decode_str(name)] = value
            header = header[size:]
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
        index_pos = decode_int(header['index_pos'])
        conn_count = decode_int(header['conn_count'])
        chunk_count = decode_int(header['chunk_count'])
        self._skip_sized()
        return BagHeader(index_pos, conn_count, chunk_count)

    def _read_connection_record(self) -> ConnectionInfo:
        header = self._read_header(OpCode.CONNECTION_INFO)
        conn = self._read_header()
        callerid: Optional[str] = None
        latching: Optional[str] = None
        if 'callerid' in conn:
            callerid = decode_str(conn['callerid'])
        if 'latching' in conn:
            latching = decode_str(conn['latching'])
        return ConnectionRecord(conn=decode_str(header['conn']),
                                callerid=callerid,
                                latching=latching,
                                topic=decode_str(header['topic']),
                                topic_original=decode_str(conn['topic']),
                                typ=decode_str(conn['type']),
                                md5sum=decode_str(conn['md5sum']),
                                message_definition=decode_str(conn['message_definition']))  # noqa

    def _read_chunk_info_record(self):
        raise NotImplementedError

    def _read_chunk_record(self):
        raise NotImplementedError
