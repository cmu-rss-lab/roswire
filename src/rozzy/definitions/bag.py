__all__ = ['Bag']

from typing import Dict
from io import BytesIO


# TODO make immutable
class RecordHeader:
    @classmethod
    def from_byte_stream(cls, s: BytesIO) -> 'BagRecord':
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

    def __getitem__(self, name: str) -> bytes:
        """
        Retrieves the value of a given field.
        """
        return self.__fields[name]


class BagRecord:
    @classmethod
    def from_byte_stream(cls, s: BytesIO) -> 'BagRecord':
        header = RecordHeader.from_byte_stream(s)
        return cls.from_byte_stream_with_header(s, header)

    @classmethod
    def from_byte_stream_with_header(cls,
                                     s: BytesIO,
                                     header: RecordHeader
                                     ) -> 'BagRecord':
        raise NotImplementedError

    def __init__(self, header: RecordHeader) -> None:
        self._header = header

    @property
    def header(self) -> RecordHeader:
        return self._header


class BagHeaderRecord(BagRecord):
    @classmethod
    def from_byte_stream_with_header(cls,
                                     s: BytesIO,
                                     header: RecordHeader
                                     ) -> 'BagHeaderRecord':
        assert header['op'] == b'\x03'
        len_padding = 4096 - header.size
        s.read(len_padding)
        return BagHeaderRecord(header)

    def __init__(self, header: RecordHeader) -> None:
        super().__init__(header)
        self.__index_pos = int.from_bytes(header['index_pos'], 'little')
        self.__conn_count = int.from_bytes(header['conn_count'], 'little')
        self.__chunk_count = int.from_bytes(header['chunk_count'], 'little')

    @property
    def index_pos(self) -> int:
        """
        Offset of the first record after the chunk section.
        """
        return self.__index_pos

    @property
    def conn_count(self) -> int:
        """
        Number of unique connections in the file.
        """
        return self.__conn_count

    @property
    def chunk_count(self) -> int:
        """
        Number of chunk records in the file.
        """
        return self.__chunk_count


class ChunkRecord(BagRecord):
    @classmethod
    def from_byte_stream_with_header(cls,
                                     s: BytesIO,
                                     header: RecordHeader
                                     ) -> 'ChunkRecord':
        assert header['op'] == b'\x05'
        return

    def __init__(self,
                 header: RecordHeader,
                 data: Sequence[MessageDataRecord, ConnectionRecord]
                 ) -> None:
        self.__compression: str = header['compression'].decode('utf-8')
        self.__size = int.from_bytes(header['size'], 'little')

    @property
    def compression(self) -> str:
        """
        The type of compression used by this chunk.
        """
        return self.__compression

    @property
    def size(self) -> int:
        """
        The size, in bytes, of the uncompressed chunk.
        """
        return self.__size


class ConnectionRecord(BagRecord):
    pass


class MessageDataRecord(BagRecord):
    pass


class IndexDataRecord(BagRecord):
    pass


class ChunkInfoRecord(BagRecord):
    pass


class Bag:
    @staticmethod
    def from_byte_stream(s: BytesIO) -> 'Bag':
        version_line: str = s.readline().decode('utf-8')
        assert version_line == '#ROSBAG V2.0\n'

        header = BagHeaderRecord.from_byte_stream(s)

        return Bag(header)

    @staticmethod
    def from_file(fn: str) -> 'Bag':
        with open(fn, 'rb') as f:
            return Bag.from_byte_stream(f)

    def __init__(self, header: BagHeaderRecord) -> None:
        self.__header = header

    @property
    def header(self) -> BagHeaderRecord:
        return self.__header
