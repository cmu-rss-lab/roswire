import os
import logging
import tempfile

import yaml
import pytest

from roswire.bag import BagReader, BagWriter
from roswire.description import SystemDescription
from roswire.definitions import TypeDatabase, FormatDatabase, PackageDatabase

from test_basic import build_ardu

DIR_TEST = os.path.dirname(__file__)


def load_mavros_type_db() -> TypeDatabase:
    fn_db_format = os.path.join(DIR_TEST,
                                'format-databases/mavros.formats.yml')
    db_format = FormatDatabase.load(fn_db_format)
    return TypeDatabase.build(db_format)


def load_mavros_description() -> SystemDescription:
    fn_db_format = os.path.join(DIR_TEST,
                                'format-databases/mavros.formats.yml')
    db_format = FormatDatabase.load(fn_db_format)
    db_type = TypeDatabase.build(db_format)
    desc = SystemDescription(sha256='foo',
                         types=db_type,
                         formats=db_format,
                         packages=PackageDatabase([]))
    return desc


def check_example_bag(db_type: TypeDatabase, bag: BagReader) -> None:
    typ_mavlink = db_type['mavros_msgs/Mavlink']
    assert bag.header.index_pos == 189991
    assert bag.header.conn_count == 7
    assert bag.header.chunk_count == 1
    assert bag.topics == {'/rosout', '/mavros/mission/waypoints',
                          '/mavlink/from', '/diagnostics', '/rosout_agg',
                          '/mavros/state'}

    msgs = list(bag.read_messages(['/mavlink/from']))
    assert all(m.topic == '/mavlink/from' for m in msgs)
    assert all(isinstance(m.message, typ_mavlink) for m in msgs)


def test_from_file():
    db_type = load_mavros_type_db()
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag = BagReader(fn_bag, db_type)
    check_example_bag(db_type, bag)


def test_write():
    db_type = load_mavros_type_db()

    # load the messages from the example bag
    reader = BagReader(os.path.join(DIR_TEST, 'example.bag'), db_type)
    messages = list(reader)

    fn_bag = tempfile.mkstemp(suffix='.bag')[1]
    try:
        bag = BagWriter(fn_bag)
        bag.write(messages)
        bag.close()

        reader = BagReader(fn_bag, db_type)
        check_example_bag(db_type, reader)
    finally:
        os.remove(fn_bag)


def test_simple_write():
    db_type = load_mavros_type_db()

    # load the messages from the example bag
    reader = BagReader(os.path.join(DIR_TEST, 'simple.bag'), db_type)
    messages = list(reader)

    fn_bag = tempfile.mkstemp(suffix='.bag')[1]
    try:
        bag = BagWriter(fn_bag)
        bag.write(messages)
        bag.close()

        reader = BagReader(fn_bag, db_type)
        assert reader.header.chunk_count == 1
        assert reader.header.conn_count == 1
        assert reader.header.topics == {'/hello'}
        msgs = list(bag.read_messages('/hello'))
        assert all(isinstance(m.message, str) for m in msgs)
    finally:
        os.remove(fn_bag)
