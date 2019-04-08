import os

import pytest

from roswire.definitions.bag import BagReader

DIR_TEST = os.path.dirname(__file__)


# @pytest.mark.skip(reason='not implemented')
def test_from_file():
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag = BagReader(fn_bag)
    print(f"Index Pos: {bag.header.index_pos}")
    print(f"Conn. Count: {bag.header.conn_count}")
    print(f"Chunk Count: {bag.header.chunk_count}")
    # print(bag.index)
    # print(bag.connections)
    print(bag.read_messages(['/mavros/state']))
    assert false
