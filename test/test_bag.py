import os
import logging
import tempfile

import yaml
import pytest

from roswire import ROSWire
from roswire.bag import BagReader, BagWriter
from roswire.description import SystemDescription
from roswire.definitions import TypeDatabase, FormatDatabase, PackageDatabase

from test_basic import build_ardu, build_hello_world

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
    assert bag.topics == {'/rosout', '/mavros/mission/waypoints',
                          '/mavlink/from', '/diagnostics', '/rosout_agg',
                          '/mavros/state'}
    assert bag.header.chunk_count == 1
    assert bag.header.conn_count == 6
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
        assert reader.topics == {'/hello'}
        msgs = list(reader.read_messages('/hello'))
    finally:
        os.remove(fn_bag)


def test_bag_replay():
    fn_bag = os.path.join(DIR_TEST, 'hello_world/bug.bag')
    with build_hello_world() as (sut, ros):
        with ros.playback(fn_bag) as player:
            player.wait()
            popen = player._process
            out = '\n'.join(popen.stream)
            assert 'error' not in out
            assert 'Done' in out


def test_write_and_replay():
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('roswire.bag.writer').addHandler(log_to_stdout)
    with build_hello_world() as (sut, ros):
        fn_bag_orig = os.path.join(DIR_TEST, 'hello_world/non-bug.bag')
        db_type = sut.description.types
        reader = BagReader(fn_bag_orig, db_type)
        messages = list(reader)

        fn_bag = 'temp.bag'
        writer = BagWriter(fn_bag)
        writer.write(messages)
        writer.close()

        ros.launch('/ros_ws/src/ros_tutorials/roscpp_tutorials/launch/listener.launch')
        with ros.playback(fn_bag) as player:
            player.wait()
            popen = player._process
            out = '\n'.join(popen.stream)
            print(out)
            assert 'error' not in out
            assert 'Done' in out
