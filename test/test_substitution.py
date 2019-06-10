import functools

import pytest

from roswire.proxy.substitution import resolve

from test_basic import build_file_and_shell_proxy


def test_resolve():
    with build_file_and_shell_proxy() as (files, shell):
        r = functools.partial(resolve, files, shell)
        assert r('hello world') == 'hello world'
