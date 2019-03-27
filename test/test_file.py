from typing import Iterator
import contextlib
import tempfile
import shutil
import os

import pytest

from roswire.proxy import FileProxy

from test_basic import build_file_proxy


def test_exists():
    with build_file_proxy() as files:
        assert not files.exists('/foo')
        assert files.exists('/tmp')
        assert files.exists('/ros_ws/entrypoint.sh')
        assert not files.exists('/ros_ws/entrypoint.shhhhh')


def test_isfile():
    with build_file_proxy() as files:
        assert not files.isfile('/foo')
        assert not files.isfile('/tmp')
        assert files.isfile('/ros_ws/entrypoint.sh')


def test_isdir():
    with build_file_proxy() as files:
        assert not files.isdir('/foo')
        assert files.isdir('/tmp')
        assert files.isdir('/ros_ws')
        assert files.isdir('/ros_ws/')
        assert not files.isdir('/ros_ws/entrypoint.sh')
        assert not files.isdir('/ros_ws/entrypoint.sh/')


def test_islink():
    with build_file_proxy() as files:
        assert files.isfile('/ros_ws/src/ArduPilot/libraries/AP_HAL_F4Light/sbus.cpp')
        assert files.islink('/ros_ws/src/ArduPilot/libraries/AP_HAL_F4Light/sbus.cpp')
        assert files.isfile('/ros_ws/src/ArduPilot/libraries/AP_HAL_Linux/sbus.cpp')
        assert not files.islink('/ros_ws/src/ArduPilot/libraries/AP_HAL_Linux/sbus.cpp')


def test_listdir():
    with build_file_proxy() as files:
        assert files.listdir('/ros_ws') == [
            '.catkin_tools',
            'build',
            'devel',
            'entrypoint.sh',
            'logs',
            'pkgs.rosinstall',
            'src'
        ]

        # not a directory
        with pytest.raises(OSError):
            files.listdir('/ros_ws/pkgs.rosinstall')

        # not a file or directory
        with pytest.raises(OSError):
            files.listdir('/ros_ws/idontexist')

        # abs
        assert files.listdir('/ros_ws', absolute=True) == [
            '/ros_ws/.catkin_tools',
            '/ros_ws/build',
            '/ros_ws/devel',
            '/ros_ws/entrypoint.sh',
            '/ros_ws/logs',
            '/ros_ws/pkgs.rosinstall',
            '/ros_ws/src'
        ]


def test_mkdir():
    with build_file_proxy() as files:
        files.mkdir('/ros_ws/cool')
        assert files.isdir('/ros_ws/cool')
        assert 'cool' in files.listdir('/ros_ws')

        # directory already exists
        with pytest.raises(FileExistsError):
            files.mkdir('/ros_ws/cool')
        assert files.isdir('/ros_ws/cool')

        # intermediate directory doesn't exist
        with pytest.raises(FileNotFoundError):
            files.mkdir('/ros_ws/foo/bar')
        assert not files.isdir('/ros_ws/foo/bar')
        assert not files.isdir('/ros_ws/foo')

        # parent directory is a file
        with pytest.raises(NotADirectoryError):
            files.mkdir('/ros_ws/entrypoint.sh/foo')
        assert not files.exists('/ros_ws/entrypoint.sh/foo')
        assert not files.isdir('/ros_ws/entrypoint.sh')
        assert files.isfile('/ros_ws/entrypoint.sh')


def test_makedirs():
    with build_file_proxy() as files:
        # parent directory exists
        files.makedirs('/ros_ws/cool')
        assert files.isdir('/ros_ws/cool')

        # intermediate directory doesn't exist
        files.makedirs('/ros_ws/foo/bar')
        assert files.isdir('/ros_ws/foo/bar')

        # directory already exists
        with pytest.raises(FileExistsError):
            files.makedirs('/ros_ws/cool')
        assert files.isdir('/ros_ws/cool')
        files.makedirs('/ros_ws/cool', exist_ok=True)

        # path is a file
        with pytest.raises(FileExistsError):
            files.makedirs('/ros_ws/entrypoint.sh')
        with pytest.raises(FileExistsError):
            files.makedirs('/ros_ws/entrypoint.sh', exist_ok=True)

        # parent directory is a file
        with pytest.raises(NotADirectoryError):
            files.makedirs('/ros_ws/entrypoint.sh/foo')
        assert not files.exists('/ros_ws/entrypoint.sh/foo')
        assert not files.isdir('/ros_ws/entrypoint.sh')
        assert files.isfile('/ros_ws/entrypoint.sh')


def test_remove():
    with build_file_proxy() as files:
        assert files.isfile('/ros_ws/pkgs.rosinstall')
        files.remove('/ros_ws/pkgs.rosinstall')
        assert not files.exists('/ros_ws/pkgs.rosinstall')

        # remove non-existent file
        with pytest.raises(FileNotFoundError):
            files.remove('/foo/bar')

        # remove directory
        assert files.isdir('/ros_ws/build')
        with pytest.raises(IsADirectoryError):
            files.remove('/ros_ws/build')
        assert files.isdir('/ros_ws/build')


def test_rmdir():
    with build_file_proxy() as files:
        # create and remove an empty directory
        files.mkdir('/tmp/foo')
        assert 'foo' in files.listdir('/tmp')
        assert files.isdir('/tmp/foo')
        files.rmdir('/tmp/foo')
        assert not files.exists('/tmp/foo')

        # remove a file
        assert files.isfile('/ros_ws/pkgs.rosinstall')
        with pytest.raises(NotADirectoryError):
            files.rmdir('/ros_ws/pkgs.rosinstall')
        assert files.isfile('/ros_ws/pkgs.rosinstall')

        # remove a non-existent file/directory
        assert not files.exists('/tmp/foo')
        with pytest.raises(FileNotFoundError):
            files.rmdir('/tmp/foo')

        # remove a non-empty directory
        assert files.isdir('/ros_ws/src')
        with pytest.raises(OSError):
            files.rmdir('/ros_ws/src')
        assert files.isdir('/ros_ws/src')
        assert files.isdir('/ros_ws/src/ArduPilot')


def test_rmtree():
    with build_file_proxy() as files:
        # create and remove an empty directory
        files.mkdir('/tmp/foo')
        assert files.isdir('/tmp/foo')
        files.rmtree('/tmp/foo')
        assert not files.exists('/tmp/foo')

        # remove a file
        assert files.isfile('/ros_ws/pkgs.rosinstall')
        with pytest.raises(NotADirectoryError):
            files.rmdir('/ros_ws/pkgs.rosinstall')
        assert files.isfile('/ros_ws/pkgs.rosinstall')

        # remove a non-existent file/directory
        assert not files.exists('/tmp/foo')
        with pytest.raises(FileNotFoundError):
            files.rmdir('/tmp/foo')

        # remove a non-empty directory
        assert files.isdir('/ros_ws/src')
        files.rmtree('/ros_ws/src')
        assert not files.exists('/ros_ws/src')


def test_copy_to_host():
    with build_file_proxy() as files:
        # copy file
        expected = """
#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
""".lstrip()
        _, fn_host = tempfile.mkstemp()
        try:
            files.copy_to_host('/ros_entrypoint.sh', fn_host)
            with open(fn_host, 'r') as f:
                contents = f.read()
            assert contents == expected
        finally:
            os.remove(fn_host)

        # non-existent file
        with pytest.raises(FileNotFoundError):
            files.copy_to_host('/foo/bar', fn_host)

        # copy directory
        dir_host: str = tempfile.mkdtemp()
        dir_host_src: str = os.path.join(dir_host, 'src')
        try:
            files.copy_to_host('/ros_ws/src/geometry/eigen_conversions/src',
                               dir_host)
            assert os.path.isdir(dir_host_src)
            assert set(os.listdir(dir_host_src)) == {'eigen_kdl.cpp', 'eigen_msg.cpp'}
        finally:
            shutil.rmtree(dir_host)


def test_copy_from_host():
    with build_file_proxy() as files:
        # create temporary file on host
        expected = """
#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
""".lstrip()
        _, fn_host = tempfile.mkstemp()
        try:
            with open(fn_host, 'w') as f:
                f.write(expected)

            fn_container = '/tmp/foo'
            files.copy_from_host(fn_host, fn_container)
            assert files.isfile(fn_container)
            assert files.read(fn_container) == expected
            files.remove(fn_container)
        finally:
            os.remove(fn_host)

        # copy non-existent file
        assert not os.path.exists(fn_host)
        with pytest.raises(FileNotFoundError):
            files.copy_from_host(fn_host, '/tmp/bar')

        # copy directory
        dir_host = tempfile.mkdtemp()
        dir_container = '/tmp/cool'
        try:
            _, fn_host_foo = tempfile.mkstemp(dir=dir_host)
            fn_container_foo = os.path.join(dir_container,
                                            os.path.basename(fn_host_foo))
            with open(fn_host_foo, 'w') as f:
                f.write(expected)

            files.copy_from_host(dir_host, dir_container)
            assert files.isdir(dir_container)
            assert files.isfile(fn_container_foo)
            assert files.read(fn_container_foo) == expected
        finally:
            shutil.rmtree(dir_host)


def test_read():
    with build_file_proxy() as files:
        # read file
        expected = "geometry_msgs/TransformStamped[] transforms\n"
        assert files.read('/ros_ws/src/geometry/tf/msg/tfMessage.msg') == expected

        # read binary file
        binary = files.read('/ros_ws/src/geometry/tf/msg/tfMessage.msg', binary=True)
        text = binary.decode('utf-8')
        assert text == expected

        # non-existent file
        with pytest.raises(FileNotFoundError):
            files.read('/foo/bar')

        # directory
        with pytest.raises(IsADirectoryError):
            files.read('/ros_ws')


def test_write():
    with build_file_proxy() as files:
        # write to new file
        expected = "hello world"
        files.write('/tmp/foo', expected)
        assert files.isfile('/tmp/foo')
        assert files.read('/tmp/foo') == expected

        # overwrite existing file
        expected = "goodbye world"
        files.write('/tmp/foo', expected)
        assert files.read('/tmp/foo') == expected
        files.remove('/tmp/foo')

        # write to a file that belongs to a non-existent directory
        with pytest.raises(FileNotFoundError):
            files.write('/tmp/bar/bork', 'code things')


def test_mktemp():
    with build_file_proxy() as files:
        # create a temporary file
        fn = files.mktemp()
        assert files.isfile(fn)

        # use specified dir
        d = '/boop'
        files.makedirs(d, exist_ok=True)
        assert files.isdir(d)
        fn = files.mktemp(dirname=d)
        assert os.path.dirname(fn) == d
        assert files.isfile(fn)

        # use non-existent dir
        with pytest.raises(FileNotFoundError):
            d = '/idontexist'
            assert not files.isdir(d)
            files.mktemp(dirname=d)

        # add suffix
        fn = files.mktemp(suffix='.foo')
        assert fn.endswith('.foo')
        assert files.isfile(fn)

        # add prefix
        fn = files.mktemp(prefix='foo')
        assert os.path.basename(fn).startswith('foo')
        assert files.isfile(fn)


def test_tempfile():
    with build_file_proxy() as files:
        fn: str
        with files.tempfile() as fn:
            assert files.isfile(fn)
        assert not files.exists(fn)
