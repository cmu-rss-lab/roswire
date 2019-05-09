import pytest

from roswire.proxy import CatkinProxy

from test_basic import build_hello_world


def test_build():
    with build_hello_world() as (sut, ros):
        catkin = sut.catkin('/ros_ws')
        catkin.build()
        assert False
