import pytest

import roswire.name


def test_global_name():
    f = roswire.name.global_name
    assert f('') == '/'
    assert f('/') == '/'
    assert f('foo') == '/foo/'
    assert f('foo/') == '/foo/'
    assert f('/foo/') == '/foo/'
    assert f('foo/bar') == '/foo/bar/'
    assert f('foo/bar/') == '/foo/bar/'
    assert f('/foo/bar/') == '/foo/bar/'
