import pytest
import attr

from roswire.definitions import (PackageDatabase, FormatDatabase, TypeDatabase,
                                 MsgFormat, Time)

from test_file import build_file_proxy


def test_build():
    with build_file_proxy() as files:
        paths = [
            '/ros_ws/src/geometry2/tf2_msgs',
            '/ros_ws/src/geometry/tf',
            '/ros_ws/src/common_msgs/geometry_msgs',
            '/ros_ws/src/std_msgs'
        ]
        db_package = PackageDatabase.from_paths(files, paths)
        db_format = FormatDatabase.build(db_package)
        db_type = TypeDatabase.build(db_format)
        assert set(db_type) == {
            'geometry_msgs/Point',
            'geometry_msgs/Point32',
            'geometry_msgs/Pose2D',
            'geometry_msgs/Quaternion',
            'geometry_msgs/Vector3',
            'std_msgs/Bool',
            'std_msgs/Byte',
            'std_msgs/Char',
            'std_msgs/ColorRGBA',
            'std_msgs/Duration',
            'std_msgs/Empty',
            'std_msgs/Float32',
            'std_msgs/Float64',
            'std_msgs/Header',
            'std_msgs/Int16',
            'std_msgs/Int32',
            'std_msgs/Int64',
            'std_msgs/Int8',
            'std_msgs/MultiArrayDimension',
            'std_msgs/String',
            'std_msgs/Time',
            'std_msgs/UInt16',
            'std_msgs/UInt32',
            'std_msgs/UInt64',
            'std_msgs/UInt8',
            'tf/FrameGraphResponse',
            'tf2_msgs/FrameGraphResponse',
            'tf2_msgs/LookupTransformGoal',
            'tf2_msgs/TF2Error',
            'geometry_msgs/Accel',
            'geometry_msgs/Inertia',
            'geometry_msgs/PointStamped',
            'geometry_msgs/Polygon',
            'geometry_msgs/Pose',
            'geometry_msgs/QuaternionStamped',
            'geometry_msgs/Transform',
            'geometry_msgs/Twist',
            'geometry_msgs/Vector3Stamped',
            'geometry_msgs/Wrench',
            'std_msgs/MultiArrayLayout',
            'geometry_msgs/AccelStamped',
            'geometry_msgs/AccelWithCovariance',
            'geometry_msgs/InertiaStamped',
            'geometry_msgs/PolygonStamped',
            'geometry_msgs/PoseArray',
            'geometry_msgs/PoseStamped',
            'geometry_msgs/PoseWithCovariance',
            'geometry_msgs/TransformStamped',
            'geometry_msgs/TwistStamped',
            'geometry_msgs/TwistWithCovariance',
            'geometry_msgs/WrenchStamped',
            'std_msgs/ByteMultiArray',
            'std_msgs/Float32MultiArray',
            'std_msgs/Float64MultiArray',
            'std_msgs/Int16MultiArray',
            'std_msgs/Int32MultiArray',
            'std_msgs/Int64MultiArray',
            'std_msgs/Int8MultiArray',
            'std_msgs/UInt16MultiArray',
            'std_msgs/UInt32MultiArray',
            'std_msgs/UInt64MultiArray',
            'std_msgs/UInt8MultiArray',
            'geometry_msgs/AccelWithCovarianceStamped',
            'geometry_msgs/PoseWithCovarianceStamped',
            'geometry_msgs/TwistWithCovarianceStamped',
            'tf/tfMessage',
            'tf2_msgs/LookupTransformResult',
            'tf2_msgs/TFMessage'
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
