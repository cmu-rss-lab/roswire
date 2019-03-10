from typing import Iterator
import contextlib
import tempfile
import shutil
import os

import pytest

from rozzy.proxy import FileProxy
from rozzy.definitions import MsgFormat, SrvFormat, Package

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
