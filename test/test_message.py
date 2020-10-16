# -*- coding: utf-8 -*-
import pytest
import roswire
from roswire.definitions import Time


@pytest.mark.parametrize('app', ['fetch'], indirect=True)
def test_encode_and_decode(app: roswire.App):
    Header = app.description.types['std_msgs/Header']
    orig = Header(seq=32, stamp=Time(secs=9781, nsecs=321), frame_id='')
    assert orig == Header.decode(orig.encode())


@pytest.mark.parametrize('app', ['fetch'], indirect=True)
def test_to_and_from_dict(app: roswire.App):
    Quaternion = app.description.types['geometry_msgs/Quaternion']
    Point = app.description.types['geometry_msgs/Point']
    Pose = app.description.types['geometry_msgs/Pose']
    orientation = Quaternion(x=0.5, y=1.0, z=2.0, w=3.0)
    position = Point(x=10.0, y=20.0, z=30.0)
    pose = Pose(position=position, orientation=orientation)
    assert pose.to_dict() == {
        'orientation': {
            'x': orientation.x,
            'y': orientation.y,
            'z': orientation.z,
            'w': orientation.w},
        'position': {
            'x': position.x,
            'y': position.y,
            'z': position.z
        }}
