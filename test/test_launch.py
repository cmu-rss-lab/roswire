import pytest

from roswire.proxy.launch import LaunchFileReader

from test_basic import build_file_and_shell_proxy


@pytest.mark.skip(reason='references a non-existent launch file')
def test_read():
    with build_file_and_shell_proxy() as (shell, files):
        reader = LaunchFileReader(files, shell)
        reader.read('/ros_ws/src/mavros/mavros/launch/apm.launch',
                    argv=['foo:=bar', 'snoot:=boop'])
        assert False
