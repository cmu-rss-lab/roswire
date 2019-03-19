import os

from rozzy.definitions.bag import BagReader

DIR_TEST = os.path.dirname(__file__)


def test_from_file():
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag = BagReader.from_file(fn_bag)
    print(f"Index Pos: {bag.header.index_pos}")
    print(f"Conn. Count: {bag.header.conn_count}")
    print(f"Chunk Count: {bag.header.chunk_count}")
    assert false
