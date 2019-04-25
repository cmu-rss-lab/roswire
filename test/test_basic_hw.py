from typing import Iterator, Tuple
import contextlib
import logging
import time

import pytest

import roswire
import roswire.exceptions
from roswire import ROSWire, ROSProxy, System, SystemDescription
from roswire.proxy import ShellProxy, FileProxy, ContainerProxy

NAME_IMAGE = 'hello-world'


def test_description():
    rsw = ROSWire()
    desc = rsw.descriptions.build(NAME_IMAGE, save=False)
