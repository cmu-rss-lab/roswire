from typing import Iterator, Tuple
import contextlib
import logging
import time

import pytest

import roswire
import roswire.exceptions
from roswire import ROSWire, ROSProxy, System, SystemDescription
from roswire.proxy import ShellProxy, FileProxy, ContainerProxy


@contextlib.contextmanager
def build_test_environment() -> Iterator[Tuple[System, ROSProxy]]:
    rsw = ROSWire()
    image = 'hello-world'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        with sut.roscore() as ros:
            time.sleep(5)
            yield (sut, ros)


@contextlib.contextmanager
def build_hw() -> Iterator[Tuple[System, ROSProxy]]:
    rsw = ROSWire()
    with rsw.launch('hello-world') as sut:
        with sut.roscore() as ros:
            time.sleep(5)
            yield (sut, ros)


@contextlib.contextmanager
def build_shell_proxy() -> Iterator[ShellProxy]:
    rsw = ROSWire()
    image = 'hello-world'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.shell


@contextlib.contextmanager
def build_file_proxy() -> Iterator[FileProxy]:
    rsw = ROSWire()
    image = 'hello-world'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.files


def test_dumb():
    with build_test_environment() as (sut, ros):
        print(ros.topic_to_type)


if __name__ == '__main__':
    test_dumb()
