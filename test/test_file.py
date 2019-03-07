from typing import Iterator
import contextlib

from test_basic import build_test_environment

from rozzy.proxy import FileProxy


@contextlib.contextmanager
def build_file_proxy() -> Iterator[FileProxy]:
    with build_test_environment() as (container, ros):
        yield container.files


def test_isfile():
    with build_file_proxy() as files:
        assert not files.isfile('/foo')
        assert not files.isfile('/tmp')
        assert files.isfile('/ros_ws/entrypoint.sh')


def test_exists():
    with build_file_proxy() as files:
        assert not files.exists('/foo')
        assert files.exists('/tmp')
        assert files.exists('/ros_ws/entrypoint.sh')
        assert not files.exists('/ros_ws/entrypoint.shhhhh')
