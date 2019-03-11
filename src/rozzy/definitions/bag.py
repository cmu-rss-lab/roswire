__all__ = ['Bag']

import io


class BagRecord:
    pass


class BagHeaderRecord(BagRecord):
    pass


class ChunkRecord(BagRecord):
    pass


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
    def from_byte_stream(f: io.BytesIO) -> 'Bag':
        version_line: str = f.readline().decode('utf-8').strip()
        assert version_line == '#ROSBAG V2.0'

    @staticmethod
    def from_bytes(b: bytes) -> 'Bag':
        return Bag.from_byte_stream(io.BytesIO(b))

    @staticmethod
    def from_file(fn: str) -> 'Bag':
        with open(fn, 'rb') as f:
            return Bag.from_byte_stream(f)
