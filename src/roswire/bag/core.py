__all__ = ('OpCode', 'Compression', 'BagMessage', 'ChunkConnection', 'Chunk',
           'ConnectionInfo', 'BagHeader', 'IndexEntry', 'Index')

from typing import Dict, Optional, Tuple, List
from enum import Enum

import attr

from ..definitions.base import Time
from ..definitions.msg import Message


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


class Compression(Enum):
    NONE = 'none'
    BZ2 = 'bz2'


@attr.s(frozen=True, slots=True)
class BagMessage:
    topic: str = attr.ib()
    time: Time = attr.ib()
    message: Message = attr.ib()


@attr.s(frozen=True, slots=True)
class ChunkConnection:
    # connection id
    uid: int = attr.ib()
    # number of messages that arrived on this connection in the chunk
    count: int = attr.ib()


@attr.s(frozen=True, slots=True)
class Chunk:
    pos_record: int = attr.ib()
    pos_data: int = attr.ib()
    time_start: Time = attr.ib()
    time_end: Time = attr.ib()
    connections: Tuple[ChunkConnection, ...] = attr.ib(converter=tuple)
    compression: Compression = attr.ib()
    size_uncompressed: int = attr.ib()
    size_compressed: int = attr.ib()


@attr.s(frozen=True, slots=True)
class ConnectionInfo:
    conn: int = attr.ib()
    topic: str = attr.ib()
    topic_original: str = attr.ib()
    typ: str = attr.ib()
    md5sum: str = attr.ib()
    message_definition: str = attr.ib()
    callerid: Optional[str] = attr.ib()
    latching: Optional[str] = attr.ib()


@attr.s(frozen=True, slots=True)
class BagHeader:
    index_pos: int = attr.ib()
    conn_count: int = attr.ib()
    chunk_count: int = attr.ib()


@attr.s(frozen=True, slots=True)
class IndexEntry:
    time: Time = attr.ib()
    pos: int = attr.ib()
    offset: int = attr.ib()


Index = Dict[int, List[IndexEntry]]
