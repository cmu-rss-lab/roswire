from typing import Iterator, Tuple
import contextlib
import logging
import time

import pytest

import rozzy
import rozzy.exceptions
from rozzy import Rozzy, ROSProxy, Container, System


@contextlib.contextmanager
def build_test_environment() -> Iterator[Tuple[ROSProxy, Container]]:
    rozzy = Rozzy()
    system = System('brass')
    with rozzy.launch(system) as container:
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

        ros.launch('mavros', 'apm.launch', fcu_url='tcp://127.0.0.1:5760@5760')
        time.sleep(30)

        assert set(ros.nodes) == {'/mavros', '/rosout'}
        assert '/mavros' in ros.nodes
        assert '/rosout' in ros.nodes
        assert '/cool' not in ros.nodes

        with pytest.raises(rozzy.exceptions.NodeNotFoundError):
            ros.nodes['/cool']

        node_mavros = ros.nodes['/mavros']
        assert node_mavros.name == '/mavros'
        assert node_mavros.pid > 0
        print(f"URL: {node_mavros.url}")
        print(f"PID: {node_mavros.pid}")

        print(list(ros.services))
        print(ros.services['/mavros/set_mode'])
        assert '/mavros/set_mode' in ros.services

        with pytest.raises(rozzy.exceptions.ServiceNotFoundError):
            ros.services['/coolio']

        assert '/coolio' not in ros.services


if __name__ == '__main__':
    test_arducopter()
