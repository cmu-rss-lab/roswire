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


def test_name_is_private():
    f = roswire.name.name_is_private
    assert not f('/foo')
    assert not f('/foo/bar')
    assert not f('foo')
    assert not f('foo/')
    assert f('~foo')
    assert f('~bar')
