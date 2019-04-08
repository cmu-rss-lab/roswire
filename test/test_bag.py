import os

import pytest

from roswire.definitions.bag import BagReader

DIR_TEST = os.path.dirname(__file__)


def test_from_file():
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag = BagReader(fn_bag)
    print(f"Index Pos: {bag.header.index_pos}")
    print(f"Conn. Count: {bag.header.conn_count}")
    print(f"Chunk Count: {bag.header.chunk_count}")
    msgs = list(bag.read_messages(['/mavlink/from']))
    print(msgs[3])
