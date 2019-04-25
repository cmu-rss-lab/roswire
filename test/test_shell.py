import subprocess

import pytest

from roswire.proxy import ShellProxy

from test_basic import build_shell_proxy


def test_exists():
    with build_shell_proxy() as shell:
        p = shell.popen('sleep 30 && echo "hello world"')
        with pytest.raises(subprocess.TimeoutExpired):
            p.wait(1)
        assert p.wait(35) == 0

        p = shell.popen('sleep 10 && exit 1')
        assert p.wait() == 1
