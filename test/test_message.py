import pytest

from roswire.definitions import (TypeDatabase, FormatDatabase, MsgFormat,
                                 Time, Duration)

from test_bag import load_mavros_type_db


def test_encode_and_decode():
    db_type = load_mavros_type_db()
    Header = db_type['std_msgs/Header']
    orig = Header(seq=32, stamp=Time(secs=9781, nsecs=321), frame_id='')
    assert orig == Header.decode(orig.encode())
