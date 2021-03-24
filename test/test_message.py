# -*- coding: utf-8 -*-
import pytest
import roswire
from roswire.common import Time
from roswire.ros2 import ROS2MsgFormat
from roswire.ros2.msg import ROS2Field

from roswire.exceptions import ParsingError


@pytest.mark.parametrize("app", ["fetch"], indirect=True)
def test_encode_and_decode(app: roswire.App):
    Header = app.description.types["std_msgs/Header"]
    orig = Header(seq=32, stamp=Time(secs=9781, nsecs=321), frame_id="")
    assert orig == Header.decode(orig.encode())


@pytest.mark.parametrize("app", ["fetch"], indirect=True)
def test_to_and_from_dict(app: roswire.App):
    Quaternion = app.description.types["geometry_msgs/Quaternion"]
    Point = app.description.types["geometry_msgs/Point"]
    Pose = app.description.types["geometry_msgs/Pose"]
    orientation = Quaternion(x=0.5, y=1.0, z=2.0, w=3.0)
    position = Point(x=10.0, y=20.0, z=30.0)
    pose = Pose(position=position, orientation=orientation)
    assert pose.to_dict() == {
        "orientation": {
            "x": orientation.x,
            "y": orientation.y,
            "z": orientation.z,
            "w": orientation.w,
        },
        "position": {"x": position.x, "y": position.y, "z": position.z},
    }


def test_from_string_ros2():
    # Ensure that string is resolved as a base type
    msg_string = ("string<=256 node_namespace\n"
                  "string<=256 node_name\n"
                  "Gid[] reader_gid_seq\n"
                  "Gid[] writer_gid_seq\n")
    msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
    assert len(msg.fields) == 4
    assert ROS2Field("string", "node_namespace", None) in msg.fields
    assert ROS2Field("string", "node_name", None) in msg.fields
    assert ROS2Field("Gid[]", "reader_gid_seq", None) in msg.fields
    assert ROS2Field("Gid[]", "writer_gid_seq", None) in msg.fields

    # Ensure that non-string doesn't resolve as basetype
    msg_string = "strin<=256 field"
    msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
    # Strictly speaking, the bounds can only happen on string and wstring
    # but we don't check that
    assert ROS2Field("apackage/strin", "field", None) in msg.fields

    # Ensure that <= NaN fails to parse
    try:
        msg_string = "string<=str"
        msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
        assert False
    except ParsingError:
        # ok
        pass

    # Ensure that default value works
    msg_string = 'string field "default"'
    msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
    assert len(msg.fields) == 1
    assert ROS2Field("string", "node_namespace", "default") in msg.fields


def test_from_wstring_ros2():
    # Ensure that string is resolved as a base type
    msg_string = ("wstring<=256 node_namespace\n"
                  "wstring<=256 node_name\n"
                  "Gid[] reader_gid_seq\n"
                  "Gid[] writer_gid_seq\n")
    msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
    assert len(msg.fields) == 4
    assert ROS2Field("wstring", "node_namespace", None) in msg.fields
    assert ROS2Field("wstring", "node_name", None) in msg.fields
    assert ROS2Field("Gid[]", "reader_gid_seq", None) in msg.fields
    assert ROS2Field("Gid[]", "writer_gid_seq", None) in msg.fields

    # Ensure that non-string doesn't resolve as basetype
    msg_string = "wstrin<=256 field"
    msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
    # Strictly speaking, the bounds can only happen on string and wstring
    # but we don't check that
    assert ROS2Field("apackage/wstrin", "field", None) in msg.fields

    # Ensure that <= NaN fails to parse
    try:
        msg_string = "wstring<=str"
        msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
        assert False
    except ParsingError:
        # ok
        pass

    # Ensure that default value works
    msg_string = 'wstring field "default"'
    msg = ROS2MsgFormat.from_string("apackage", "amsg", msg_string)
    assert len(msg.fields) == 1
    assert ROS2Field("wstring", "node_namespace", "default") in msg.fields
