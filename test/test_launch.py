from roswire.proxy.launch import LaunchFileReader

from test_basic import build_file_and_shell_proxy


def test_read():
    with build_file_and_shell_proxy() as (files, shell):
        reader = LaunchFileReader(shell, files)
        reader.read('/ros_ws/src/mavros/mavros/launch/apm.launch')
        assert False
