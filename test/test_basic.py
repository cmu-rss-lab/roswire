from typing import Iterator, Tuple
import contextlib
import logging
import time

import pytest

from rozzy import Rozzy, ROSProxy, Container


@contextlib.contextmanager
def build_test_environment() -> Iterator[Tuple[ROSProxy, Container]]:
    rozzy = Rozzy()
    with rozzy.launch() as container:
        with container.roscore() as ros:
            time.sleep(5)
            yield (container, ros)


def test_parameters():
    with build_test_environment() as (container, ros):
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


def test_arducopter():
    logging.basicConfig()
    with build_test_environment() as (container, ros):
        cmd = ' '.join([
            "/ros_ws/src/ArduPilot/build/sitl/bin/arducopter",
            "--model copter"
        ])
        container.shell.non_blocking_execute(cmd)

        # launch mavros
        cmd = ' '.join([
            "roslaunch",
            "mavros apm.launch",
            "fcu_url:=tcp://127.0.0.1:5760@5760"
        ])
        container.shell.non_blocking_execute(cmd)

        time.sleep(30)
