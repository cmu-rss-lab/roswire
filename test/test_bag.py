import os
import logging

import pytest

from roswire.bag import BagReader

from test_basic import build_ardu

DIR_TEST = os.path.dirname(__file__)

b'\x00\x00\x00\x00J\xdb\x86\\\xa9\xa4\x11\x0e\x00\x00\x00\x00\x01\x10(\x01\x01o\xaaT\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00)@AT\x02\x00\x00\x00'


def test_from_file():
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('roswire.bag').addHandler(log_to_stdout)
    with build_ardu() as (sut, ros):
        db_type = sut.description.types
        fn_bag = os.path.join(DIR_TEST, 'example.bag')
        bag = BagReader(fn_bag, db_type)
        print(f"Index Pos: {bag.header.index_pos}")
        print(f"Conn. Count: {bag.header.conn_count}")
        print(f"Chunk Count: {bag.header.chunk_count}")
        msgs = list(bag.read_messages(['/mavlink/from']))
        print(msgs[3])
