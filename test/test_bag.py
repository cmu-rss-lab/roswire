import os
import logging

import yaml
import pytest

from roswire.bag import BagReader
from roswire.definitions import TypeDatabase, FormatDatabase

from test_basic import build_ardu

DIR_TEST = os.path.dirname(__file__)


def load_mavros_type_db() -> TypeDatabase:
    fn_db_format = os.path.join(DIR_TEST,
                                'format-databases/mavros.formats.yml')
    db_format = FormatDatabase.load(fn_db_format)
    return TypeDatabase.build(db_format)


def test_from_file():
    db_type = load_mavros_type_db()
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag = BagReader(fn_bag, db_type)
    print(f"Index Pos: {bag.header.index_pos}")
    print(f"Conn. Count: {bag.header.conn_count}")
    print(f"Chunk Count: {bag.header.chunk_count}")
    msgs = list(bag.read_messages(['/mavlink/from']))
    print(msgs[3])
