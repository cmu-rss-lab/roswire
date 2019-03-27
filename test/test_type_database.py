import pytest
import attr

from roswire.definitions import (PackageDatabase, FormatDatabase, TypeDatabase,
                                 MsgFormat, Time)

from test_file import build_file_proxy


def test_build_type():
    s = """
uint32 seq
time stamp
string frame_id
    """
    fmt = MsgFormat.from_string("PkgName", "MessageName", s)
    t = TypeDatabase.build_type(fmt)
    assert t.__name__ == fmt.name
    assert t.format == fmt

    m = t(seq=310, stamp=Time(30, 120), frame_id='foo')

    # message should be immutable
    with pytest.raises(attr.exceptions.FrozenInstanceError):
        m.seq = 500


def test_build():
    with build_file_proxy() as files:
        paths = [
            '/ros_ws/src/geometry2/tf2_msgs',
            '/ros_ws/src/geometry/tf'
        ]
        db_package = PackageDatabase.from_paths(files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)
        assert set(db_type) == {
            'tf/tfMessage',
            'tf/FrameGraphResponse',
            'tf2_msgs/TFMessage',
            'tf2_msgs/TF2Error',
            'tf2_msgs/FrameGraphResponse',
            'tf2_msgs/LookupTransformGoal',
            'tf2_msgs/LookupTransformResult'
        }

        t = db_type['tf/FrameGraphResponse']
        m = t(dot_graph='foo')
        assert m.dot_graph == 'foo'

        m = t('bar')
        assert m.dot_graph == 'bar'

        with pytest.raises(TypeError):
            t(foo='bar')
        with pytest.raises(TypeError):
            t('foo', 'bar')


def test_to_and_from_dict():
    s = """
uint32 seq
time stamp
string frame_id
    """
    fmt = MsgFormat.from_string("PkgName", "MessageName", s)
    db_fmt = FormatDatabase({fmt}, {}, {})
    db_type = TypeDatabase.build(db_fmt)

    t = db_type['PkgName/MessageName']
    m = t(seq=310, stamp=Time(30, 120), frame_id='foo')
    d = {'seq': 310,
         'stamp': {'secs': 30, 'nsecs': 120},
         'frame_id': 'foo'}
    assert m.to_dict() == d
    assert db_type.from_dict(fmt, d) == m
