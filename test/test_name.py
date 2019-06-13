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


def test_canonical_name():
    f = roswire.name.canonical_name
    assert f('') == ''
    assert f('/') == '/'
    assert f('foo') == 'foo'
    assert f('foo/') == 'foo'
    assert f('/foo') == '/foo'
    assert f('/foo/') == '/foo'
    assert f('foo/bar') == 'foo/bar'
    assert f('foo/bar/') == 'foo/bar'
    assert f('/foo/bar/') == '/foo/bar'


def test_name_is_private():
    f = roswire.name.name_is_private
    assert not f('')
    assert not f('/')
    assert not f('/foo')
    assert not f('/foo/bar')
    assert not f('foo')
    assert not f('foo/')
    assert f('~foo')
    assert f('~bar')


def test_name_is_global():
    f = roswire.name.name_is_global
    assert f('/')
    assert f('/foo')
    assert f('/foo/bar')
    assert not f('')
    assert not f('foo')
    assert not f('foo/')
    assert not f('~foo')
    assert not f('~bar')


def test_name_is_legal():
    f = roswire.name.name_is_legal
    assert f('')
    assert f('/')
    assert f('/foo')
    assert f('/foo/bar')
    assert f('/foo/bar/')
    assert f('~foo')
    assert not f('/~foo')
    assert not f('foo bar')
    assert not f('~')

def test_namespace():
    f = roswire.name.namespace
    assert f('/') == '/'
    assert f('') == '/'
    assert f('/bar') == '/'
    assert f('/bar/foo') == '/bar'
    with pytest.raises(ValueError):
        f('~foo')


def test_namespace_join():
    f = roswire.name.namespace_join
    assert f('', 'foo') == 'foo'
    assert f('', 'foo/bar') == 'foo/bar'
    assert f('/', 'foo') == '/foo'
    assert f('/foo', 'bar') == '/foo/bar'
    assert f('/foo/bar', 'beep') == '/foo/bar/beep'
    assert f('/foo/bar', '~beep') == '~beep'
    assert f('/foo/bar', '/bork') == '/bork'
