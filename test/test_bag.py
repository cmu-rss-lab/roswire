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
    typ_mavlink = db_type['mavros_msgs/Mavlink']
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag = BagReader(fn_bag, db_type)
    assert bag.header.index_pos == 189991
    assert bag.header.conn_count == 7
    assert bag.header.chunk_count == 1
    assert bag.topics == {'/rosout', '/mavros/mission/waypoints',
                          '/mavlink/from', '/diagnostics', '/rosout_agg',
                          '/mavros/state'}

    msgs = list(bag.read_messages(['/mavlink/from']))
    assert all(m.topic == '/mavlink/from' for m in msgs)
    assert all(isinstance(m.message, typ_mavlink) for m in msgs)
