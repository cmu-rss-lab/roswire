import functools

import pytest

from roswire.proxy.substitution import resolve

from test_basic import build_file_and_shell_proxy


def test_resolve():
    with build_file_and_shell_proxy() as (files, shell):
        r = functools.partial(resolve, shell, files)
        assert r('$(dirname)/foo.txt', {'filename': '/foo/bar/launch/yes.xml'}) == '/foo/bar/launch/foo.txt'
        assert r('$(find tf2)/CMakeLists.txt') == '/ros_ws/src/geometry2/tf2/CMakeLists.txt'
