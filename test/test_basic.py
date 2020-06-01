from typing import Iterator, Tuple
import os
import contextlib
import logging
import time

import pytest

import roswire
import roswire.exceptions
from dockerblade import Shell, FileSystem
from roswire import ROSWire, System, SystemDescription
from roswire.proxy import ROSCore, Container
from roswire.description import SystemDescription
from roswire.definitions import TypeDatabase, FormatDatabase, PackageDatabase

DIR_TEST = os.path.dirname(__file__)


def skip_if_on_travis(f):
    if os.environ.get('TRAVIS') == 'true':
        return pytest.mark.skipif(f, reason='skipping test on Travis')
    else:
        return f


def load_hello_world_type_db() -> TypeDatabase:
    fn_db_format = os.path.join(DIR_TEST,
                                'format-databases/helloworld.formats.yml')
    db_format = FormatDatabase.load(fn_db_format)
    return TypeDatabase.build(db_format)


def load_hello_world_description() -> SystemDescription:
    fn_db_format = os.path.join(DIR_TEST,
                                'format-databases/helloworld.formats.yml')
    db_format = FormatDatabase.load(fn_db_format)
    db_type = TypeDatabase.build(db_format)
    desc = SystemDescription(sha256='foo',
                             types=db_type,
                             formats=db_format,
                             packages=PackageDatabase([]))
    return desc


@contextlib.contextmanager
def build_ardu() -> Iterator[Tuple[System, ROSCore]]:
    rsw = ROSWire()
    with rsw.launch('brass') as sut:
        with sut.roscore() as ros:
            time.sleep(5)
            yield (sut, ros)


@contextlib.contextmanager
def build_hello_world() -> Iterator[Tuple[System, ROSCore]]:
    rsw = ROSWire()
    image = 'roswire/helloworld:buggy'
    desc = load_hello_world_description()
    with rsw.launch(image, desc) as sut:
        with sut.roscore() as ros:
            time.sleep(5)
            yield (sut, ros)


@contextlib.contextmanager
def build_shell_proxy() -> Iterator[Shell]:
    rsw = ROSWire()
    image = 'brass'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.shell


@contextlib.contextmanager
def build_file_proxy() -> Iterator[FileSystem]:
    rsw = ROSWire()
    image = 'brass'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.files


@contextlib.contextmanager
def build_file_and_shell_proxy() -> Iterator[Tuple[FileSystem, Shell]]:
    rsw = ROSWire()
    image = 'brass'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.files, sut.shell


@skip_if_on_travis
@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_parameters(sut):
    with sut.roscore() as ros:
        assert ros.topic_to_type == {'/rosout': 'rosgraph_msgs/Log',
                                     '/rosout_agg': 'rosgraph_msgs/Log'}

        assert '/rosversion' in ros.parameters
        assert '/rosdistro' in ros.parameters

        assert '/hello' not in ros.parameters
        ros.parameters['/hello'] = 'world'
        assert '/hello' in ros.parameters
        assert ros.parameters['/hello'] == 'world'

        del ros.parameters['/hello']
        assert 'hello' not in ros.parameters
        with pytest.raises(KeyError):
            ros.parameters['/hello']
