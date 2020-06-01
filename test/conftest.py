# -*- coding: utf-8 -*-
import pytest

from typing import Iterator, Sequence
import contextlib
import os

from loguru import logger
import dockerblade
import roswire
import yaml

DIR_HERE = os.path.dirname(__file__)

logger.enable('roswire')


@contextlib.contextmanager
def _sut(name: str) -> Iterator[roswire.System]:
    config_filepath = os.path.join(DIR_HERE, 'systems', f'{name}.yml')
    with open(config_filepath, 'r') as f:
        config = yaml.load(f, Loader=yaml.SafeLoader)

    image: str = config['image']
    sources: Sequence[str] = config['sources']
    # TODO load from file
    # TODO we should cache these (immutable) descriptions
    mock_description = roswire.SystemDescription(image, [], [], [], [])
    rsw = roswire.ROSWire()
    with rsw.launch(image, sources=sources, description=mock_description) as system:
        yield system


@pytest.yield_fixture
def sut(request) -> Iterator[roswire.System]:
    with _sut(request.param) as sut:
        yield sut


@pytest.yield_fixture
def filesystem(request) -> Iterator[dockerblade.files.FileSystem]:
    with _sut(request.param) as sut:
        yield sut.files


@pytest.yield_fixture
def shell(request) -> Iterator[dockerblade.shell.Shell]:
    with _sut(request.param) as sut:
        yield sut.shell
