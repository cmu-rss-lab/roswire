from roswire.proxy.launch import LaunchFileReader

from test_basic import build_file_proxy


def test_read():
    with build_file_proxy() as files:
        reader = LaunchFileReader(files)
        reader.read('/ros_ws/src/mavros/mavros/launch/apm.launch')
