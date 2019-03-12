__all__ = ['Bag']

from typing import Dict, Sequence, Union, Optional, Tuple, List
from io import BytesIO
import bz2

import attr


# TODO make immutable
class RecordHeader:
    @classmethod
    def from_stream(cls, s: BytesIO) -> 'BagRecord':
        fields: Dict[str, bytes] = {}
        size = int.from_bytes(s.read(4), 'little')
        offset: int = 4
        while offset < size:
            bytes_name: bytes
            value: bytes
            length = int.from_bytes(s.read(4), 'little')
            bytes_name, _, value = s.read(length).partition(b'\x3d')
            name: str = bytes_name.decode('utf-8')
            fields[name] = value
            offset += length
        return RecordHeader(fields, size)

    def __init__(self,
                 fields: Dict[str, bytes],
                 size: int
                 ) -> None:
        self.__fields = fields
        self.__size = size

    @property
    def fields(self) -> Dict[str, bytes]:
        return self.__fields

    @property
    def size(self) -> int:
        """
        The size of the header, measured in bytes.
        """
        return self.__size

    def __contains__(self, name: str) -> bool:
        return name in self.__fields

    def __getitem__(self, name: str) -> bytes:
        """
        Retrieves the value of a given field.
        """
        return self.__fields[name]


class BagRecord:
    @classmethod
    def from_stream(cls, s: BytesIO) -> 'BagRecord':
        header = RecordHeader.from_stream(s)
        return cls.from_stream_with_header(s, header)

    @classmethod
    def from_stream_with_header(cls,
                                s: BytesIO,
                                header: RecordHeader
                                ) -> 'BagRecord':
        raise NotImplementedError


@attr.s(frozen=True)
class BagHeaderRecord(BagRecord):
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
        return BagHeaderRecord(index_pos, conn_count, chunk_count)


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

        header = BagHeaderRecord.from_stream(s)

        return Bag(header)

    @staticmethod
    def from_file(fn: str) -> 'Bag':
        with open(fn, 'rb') as f:
            return Bag.from_stream(f)

    def __init__(self, header: BagHeaderRecord) -> None:
        self.__header = header

    @property
    def header(self) -> BagHeaderRecord:
        return self.__header
