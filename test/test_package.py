from typing import Iterator
import contextlib
import tempfile
import shutil
import os

import pytest

from roswire.proxy import FileProxy
from roswire.definitions import MsgFormat, SrvFormat, Package, PackageDatabase

from test_file import build_file_proxy
from test_basic import build_shell_proxy


def test_to_and_from_dict():
    pkg = 'tf'
    msg_tf = MsgFormat.from_dict({
        'package': pkg,
        'name': 'tfMessage',
        'fields': [{'type': 'geometry_msgs/TransformStamped[]',
                    'name': 'transforms'}]})
    srv_fg = SrvFormat.from_dict({
        'package': pkg,
        'name': 'FrameGraph',
        'response': {
            'fields': [{'type': 'string', 'name': 'dot_graph'}]}})
    p = Package(name=pkg,
                path='/ros_ws/src/geometry/tf',
                messages=[msg_tf],
                actions=[],
                services=[srv_fg])
    assert p == Package.from_dict(p.to_dict())


def test_build():
    with build_file_proxy() as files:
        path = '/ros_ws/src/geometry/tf'
        expected = Package.from_dict({
            'path': path,
            'name': 'tf',
            'messages': [
                {'name': 'tfMessage',
                 'fields': [{'type': 'geometry_msgs/TransformStamped[]',
                             'name': 'transforms'}]}
            ],
            'services': [
                {'name': 'FrameGraph',
                 'response': {
                 'fields': [{'type': 'string',
                             'name': 'dot_graph'}]}}
            ]
        })
        actual = Package.build(path, files)
        assert actual == expected


def test_database_paths():
    with build_shell_proxy() as shell:
        expected = [
            '/ros_ws/src/catkin',
            '/ros_ws/src/genmsg',
            '/ros_ws/src/gencpp',
            '/ros_ws/src/genlisp',
            '/ros_ws/src/genpy',
            '/ros_ws/src/cmake_modules',
            '/ros_ws/src/class_loader',
            '/ros_ws/src/roscpp_core/cpp_common',
            '/ros_ws/src/mavlink',
            '/ros_ws/src/mavros/libmavconn',
            '/ros_ws/src/message_generation',
            '/ros_ws/src/message_runtime',
            '/ros_ws/src/orocos_kinematics_dynamics/orocos_kdl',
            '/ros_ws/src/ros/rosbuild',
            '/ros_ws/src/ros/rosclean',
            '/ros_ws/src/roscpp_core/roscpp_traits',
            '/ros_ws/src/ros_comm/rosgraph',
            '/ros_ws/src/ros/roslang',
            '/ros_ws/src/ros_comm/rosmaster',
            '/ros_ws/src/ros_comm/rosmsg',
            '/ros_ws/src/rospack',
            '/ros_ws/src/ros/roslib',
            '/ros_ws/src/ros_comm/rosparam',
            '/ros_ws/src/ros_comm/rospy',
            '/ros_ws/src/ros_comm/rosservice',
            '/ros_ws/src/roscpp_core/rostime',
            '/ros_ws/src/roscpp_core/roscpp_serialization',
            '/ros_ws/src/ros_comm/roslaunch',
            '/ros_ws/src/ros/rosunit',
            '/ros_ws/src/angles',
            '/ros_ws/src/ros_comm/rosconsole',
            '/ros_ws/src/pluginlib',
            '/ros_ws/src/rosconsole_bridge',
            '/ros_ws/src/ros_comm/roslz4',
            '/ros_ws/src/ros_comm/rosbag_storage',
            '/ros_ws/src/ros_comm/rostest',
            '/ros_ws/src/std_msgs',
            '/ros_ws/src/common_msgs/actionlib_msgs',
            '/ros_ws/src/common_msgs/diagnostic_msgs',
            '/ros_ws/src/common_msgs/geometry_msgs',
            '/ros_ws/src/geometry/eigen_conversions',
            '/ros_ws/src/mavros/mavros_msgs',
            '/ros_ws/src/common_msgs/nav_msgs',
            '/ros_ws/src/ros_comm_msgs/rosgraph_msgs',
            '/ros_ws/src/common_msgs/sensor_msgs',
            '/ros_ws/src/ros_comm_msgs/std_srvs',
            '/ros_ws/src/geometry2/tf2_msgs',
            '/ros_ws/src/geometry2/tf2',
            '/ros_ws/src/urdf/urdf_parser_plugin',
            '/ros_ws/src/common_msgs/visualization_msgs',
            '/ros_ws/src/ros_comm/xmlrpcpp',
            '/ros_ws/src/ros_comm/roscpp',
            '/ros_ws/src/ros_comm/rosout',
            '/ros_ws/src/vision_opencv/cv_bridge',
            '/ros_ws/src/diagnostics/diagnostic_updater',
            '/ros_ws/src/ros_comm/message_filters',
            '/ros_ws/src/image_common/image_transport',
            '/ros_ws/src/ros_comm/rosnode',
            '/ros_ws/src/ros_comm/rostopic',
            '/ros_ws/src/ros_comm/roswtf',
            '/ros_ws/src/geometry2/tf2_py',
            '/ros_ws/src/ros_comm/topic_tools',
            '/ros_ws/src/ros_comm/rosbag',
            '/ros_ws/src/actionlib',
            '/ros_ws/src/geometry2/tf2_ros',
            '/ros_ws/src/mavros/mavros',
            '/ros_ws/src/geometry/tf',
            '/ros_ws/src/urdf/urdf',
            '/ros_ws/src/mavros/mavros_extras',
            '/opt/ros/indigo/share',
            '/opt/ros/indigo/stacks'
        ]
        actual = PackageDatabase.paths(shell)
        assert actual == expected


def test_database_from_paths():
    with build_file_proxy() as files:
        paths = [
            '/ros_ws/src/angles',
            '/ros_ws/src/mavros',
            '/ros_ws/src/geometry2/tf2',
            '/ros_ws/src/geometry2/tf2_msgs',
            '/ros_ws/src/geometry2/tf2_py',
            '/ros_ws/src/geometry2/tf2_ros'
        ]
        db = PackageDatabase.from_paths(files, paths)
        assert len(db) == len(paths)
        assert set(db) == {'angles', 'mavros', 'tf2',
                           'tf2_msgs', 'tf2_py', 'tf2_ros'}
