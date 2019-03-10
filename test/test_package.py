from typing import Iterator
import contextlib
import tempfile
import shutil
import os

import pytest

from rozzy.proxy import FileProxy
from rozzy.definitions import MsgFormat, SrvFormat, Package, PackageDatabase

from test_file import build_file_proxy


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
        db = PackageDatabase.from_paths(paths, files)
        assert len(db) == len(paths)
        assert set(db) == {'angles', 'mavros', 'tf2',
                           'tf2_msgs', 'tf2_py', 'tf2_ros'}
