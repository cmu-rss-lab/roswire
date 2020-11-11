# -*- coding: utf-8 -*-
__all__ = (
    "OpCode",
    "Compression",
    "BagMessage",
    "ChunkConnection",
    "Chunk",
    "ConnectionInfo",
    "BagHeader",
    "IndexEntry",
    "Index",
)

from enum import Enum
from typing import Dict, List, Optional, Tuple

import attr

from ...common.base import Time
from ...common.msg import Message
from ...util import tuple_from_iterable


class OpCode(Enum):
    MESSAGE_DATA = b"\x02"
    HEADER = b"\x03"
    INDEX_DATA = b"\x04"
    CHUNK = b"\x05"
    CHUNK_INFO = b"\x06"
    CONNECTION_INFO = b"\x07"

    @property
    def hex(self) -> str:
        return f"0x{self.value.hex()}"


class Compression(Enum):
    NONE = "none"
    BZ2 = "bz2"


@attr.s(frozen=True, slots=True, auto_attribs=True)
class BagMessage:
    topic: str
    time: Time
    message: Message


@attr.s(frozen=True, slots=True, auto_attribs=True)
class ChunkConnection:
    """
    Attributes
    ----------
    uid: str
        The connection identifier
    count: int
        The number of messages that arrived on this connection in the chunk
    """

    uid: int
    count: int


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Chunk:
    pos_record: int
    pos_data: int
    time_start: Time
    time_end: Time
    connections: Tuple[ChunkConnection, ...] = attr.ib(
        converter=tuple_from_iterable
    )
    compression: Compression
    size_uncompressed: int
    size_compressed: int


@attr.s(frozen=True, slots=True, auto_attribs=True)
class ConnectionInfo:
    conn: int
    topic: str
    topic_original: str
    typ: str
    md5sum: str
    message_definition: str
    callerid: Optional[str]
    latching: Optional[str]


@attr.s(frozen=True, slots=True, auto_attribs=True)
class BagHeader:
    index_pos: int
    conn_count: int
    chunk_count: int


@attr.s(frozen=True, slots=True, auto_attribs=True)
class IndexEntry:
    time: Time
    pos: int
    offset: int


Index = Dict[int, List[IndexEntry]]
