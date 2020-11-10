# -*- coding: utf-8 -*-
import pytest

from typing import Iterator
import contextlib
import os

from loguru import logger
import docker
import dockerblade
import roswire
import yaml

DIR_HERE = os.path.dirname(__file__)

logger.enable("roswire")


def _app(name: str) -> roswire.App:
    rsw = roswire.ROSWire()
    filepath = os.path.join(DIR_HERE, "apps", f"{name}.yml")

    # ensure that the image for the app has been downloaded
    docker_client = docker.from_env()
    with open(filepath, "r") as f:
        contents = yaml.safe_load(f)
    image: str = contents["image"]
    try:
        docker_client.images.get(image)
    except docker.errors.ImageNotFound:
        logger.debug(f"downloading Docker image for testing: {image}")
        docker_client.images.pull(image)
        logger.debug(f"downloaded Docker image for testing: {image}")

    return rsw.load(filepath)


@contextlib.contextmanager
def _sut(name: str) -> Iterator[roswire.AppInstance]:
    app = _app(name)
    with app.launch() as app_instance:
        yield app_instance


@pytest.fixture
def app(request) -> roswire.App:
    return _app(request.param)


@pytest.yield_fixture
def sut(request) -> Iterator[roswire.AppInstance]:
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
