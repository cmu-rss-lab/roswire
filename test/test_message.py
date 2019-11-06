import pytest
import yaml

from roswire.definitions import (TypeDatabase, FormatDatabase, MsgFormat,
                                 Time, Duration)

from test_bag import load_mavros_type_db


def test_encode_and_decode():
    db_type = load_mavros_type_db()
    Header = db_type['std_msgs/Header']
    orig = Header(seq=32, stamp=Time(secs=9781, nsecs=321), frame_id='')
    assert orig == Header.decode(orig.encode())


def test_to_and_from_dict():
    db_type = load_mavros_type_db()
    Waypoint = db_type['mavros_msgs/Waypoint']
    WaypointPushRequest = db_type['mavros_msgs/WaypointPushRequest']

    wp = Waypoint(frame=0,
                  command=16,
                  is_current=False,
                  autocontinue=False,
                  param1=0.0,
                  param2=1.0,
                  param3=2.0,
                  param4=3.0,
                  x_lat=4.0,
                  y_long=5.0,
                  z_alt=6.0)
    req = WaypointPushRequest(waypoints=[wp])

    assert req.to_dict() == {'waypoints': [
        {'frame': 0,
         'command': 16,
         'is_current': False,
         'autocontinue': False,
         'param1': 0.0,
         'param2': 1.0,
         'param3': 2.0,
         'param4': 3.0,
         'x_lat': 4.0,
         'y_long': 5.0,
         'z_alt': 6.0}
    ]}
