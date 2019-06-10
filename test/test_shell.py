import subprocess
import time
import os

import pytest

from roswire.proxy import ShellProxy

from test_basic import build_shell_proxy


def test_wait():
    with build_shell_proxy() as shell:
        p = shell.popen('sleep 30 && echo "hello world"')
        with pytest.raises(subprocess.TimeoutExpired):
            p.wait(1)
        assert p.wait(35) == 0

        p = shell.popen('sleep 10 && exit 1')
        assert p.wait() == 1


def test_local_to_host_pid():
    with build_shell_proxy() as shell:
        p = shell.popen('sleep 60')
        pid_host = shell.local_to_host_pid(p.pid)
        assert pid_host is not None


def test_environ():
    with build_shell_proxy() as shell:
        assert shell.environ('ROS_PACKAGE_PATH')
