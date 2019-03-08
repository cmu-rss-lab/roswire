from typing import Iterator
import contextlib

import pytest

from rozzy.proxy import FileProxy

from test_basic import build_test_environment


@contextlib.contextmanager
def build_file_proxy() -> Iterator[FileProxy]:
    with build_test_environment() as (container, ros):
        yield container.files


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


def test_mkdir():
    with build_file_proxy() as files:
        files.mkdir('/ros_ws/cool')
        assert files.isdir('/ros_ws/cool')

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
