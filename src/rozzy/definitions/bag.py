__all__ = ['Bag']

from typing import Dict, Sequence, Union, Optional, Tuple, List, Type
from io import BytesIO
import bz2
import struct
import logging

import attr

logger: logging.Logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

RecordHeader = Dict[str, bytes]


def decode_uint32(v: bytes): return struct.unpack('<L', v)[0]
def decode_int(v: bytes): return int.from_bytes(v, 'little')
def decode_str(v: bytes): return v.decode('utf-8')


def read_sized(s: BytesIO) -> bytes:
    size = int.from_bytes(s.read(4), 'little')
    return s.read(size)


def read_header(s: BytesIO) -> RecordHeader:
    fields: RecordHeader = {}
    header = read_sized(s)
    while header:
        size = decode_int(header[:4])
        header = header[4:]
        name, sep, value = header[:size].partition(b'\x3d')
        if sep == '':
            raise Exception('error reading header field')
        fields[decode_str(name)] = value
        header = header[size:]
    return fields


class BagRecord:
    @classmethod
    def from_stream(cls, s: BytesIO) -> 'BagRecord':
        header = read_header(s)
        print(header)
        cls_decode: Type[BagRecord] = ({
            b'\x02': MessageDataRecord,
            b'\x04': IndexDataRecord,
            b'\x05': ChunkRecord,
            b'\x06': ChunkInfoRecord,
            b'\x07': ConnectionRecord,
        })[header['op']]
        return cls_decode.from_stream_with_header(s, header)

    @classmethod
    def from_stream_with_header(cls,
                                s: BytesIO,
                                header: RecordHeader
                                ) -> 'BagRecord':
        raise NotImplementedError


@attr.s(frozen=True)
class BagHeader:
    index_pos: int = attr.ib()
    conn_count: int = attr.ib()
    chunk_count: int = attr.ib()

    @classmethod
    def from_stream_with_header(cls,
                                s: BytesIO,
                                header: RecordHeader
                                ) -> 'BagHeaderRecord':
        assert header['op'] == b'\x03'
        len_padding = 4096 - header.size
        s.read(len_padding)

        index_pos = int.from_bytes(header['index_pos'], 'little')
        conn_count = int.from_bytes(header['conn_count'], 'little')
        chunk_count = int.from_bytes(header['chunk_count'], 'little')

        return BagHeader(index_pos, conn_count, chunk_count)


@attr.s(frozen=True)
class ConnectionRecord(BagRecord):
    conn: int = attr.ib()
    topic: str = attr.ib()
    topic_original: str = attr.ib()
    typ: str = attr.ib()
    md5sum: str = attr.ib()
    message_definition: str = attr.ib()
    callerid: Optional[str] = attr.ib()
    latching: Optional[str] = attr.ib()

    @classmethod
    def from_stream_with_header(cls,
                                s: BytesIO,
                                header: RecordHeader
                                ) -> 'ConnectionRecord':
        assert header['op'] == b'\x07'
        conn = RecordHeader.from_stream(s)
        callerid: Optional[str] = None
        latching: Optional[str] = None
        if 'callerid' in conn:
            callerid = conn['callerid'].decode('utf-8')
        if 'latching' in conn:
            latching = conn['latching'].decode('utf-8')
        return ConnectionRecord(conn=header['conn'].decode('utf-8'),
                                callerid=callerid,
                                latching=latching,
                                topic=header['topic'].decode('utf-8'),
                                topic_original=conn['topic'].decode('utf-8'),
                                typ=conn['type'].decode('utf-8'),
                                md5sum=conn['md5sum'].decode('utf-8'),
                                message_definition=conn['message_definition'].decode('utf-8'))  # noqa


class MessageDataRecord(BagRecord):
    pass


class IndexDataRecord(BagRecord):
    pass


class ChunkInfoRecord(BagRecord):
    pass


@attr.s(frozen=True)
class ChunkRecord(BagRecord):
    compression: str = attr.ib()
    size: int = attr.ib()
    records: Tuple[Union[MessageDataRecord, ConnectionRecord], ...] = attr.ib()  # noqa

    @classmethod
    def from_stream_with_header(cls,
                                s: BytesIO,
                                header: RecordHeader
                                ) -> 'ChunkRecord':
        assert header['op'] == b'\x05'
        compression: str = header['compression'].decode('utf-8')
        size_uncompressed = int.from_bytes(header['size'], 'little')
        assert compression in ['none', 'bz2']

        size_compressed = int.from_bytes(s.read(4), 'little')
        data_bytes = s.read(size_compressed)
        if compression == 'bz2':
            data_bytes = bz2.decompress(data_bytes)
        s = BytesIO(data_bytes)

        records: List[Union[MessageDataRecord, ConnectionRecord]] = []
        while s.tell() < size_uncompressed:
            record = BagRecord.from_stream(s)
            records.append(record)

        return ChunkRecord(compression, size_uncompressed, records)

# data: Sequence[Union[MessageDataRecord, ConnectionRecord]]


class Bag:
    @staticmethod
    def from_stream(s: BytesIO) -> 'Bag':
        version_line: str = s.readline().decode('utf-8')
        assert version_line == '#ROSBAG V2.0\n'

        # read header
        pos_start = s.tell()
        header = read_header(s)
        assert header['op'] == b'\x03'
        len_padding = s.tell() - pos_start + 4096
        if len_padding > 0:
            s.read(len_padding)

        index_pos = decode_int(header['index_pos'])
        conn_count = decode_int(header['conn_count'])
        chunk_count = decode_int(header['chunk_count'])

        logger.debug("* index position: %d", index_pos)
        logger.debug("* connection count: %d", conn_count)
        logger.debug("* chunk count: %d", chunk_count)

        size = int.from_bytes(s.read(4), 'little')
        ends_at = s.tell() + size
        logger.debug("* data size: %d bytes", size)
        logger.debug("* ends at: %d", ends_at)

        # read contents
        records: List[BagRecord] = []
        while s.tell() < ends_at:
            record = BagRecord.from_stream(s)
            records.append(records)
            assert False

        return Bag(header)

    @staticmethod
    def from_file(fn: str) -> 'Bag':
        with open(fn, 'rb') as f:
            return Bag.from_stream(f)

    def __init__(self, records: List[BagRecord]) -> None:
        self.__records = records
