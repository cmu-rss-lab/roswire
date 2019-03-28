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
    image = 'brass'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        with sut.roscore() as ros:
            time.sleep(5)
            yield (sut, ros)


@contextlib.contextmanager
def build_ardu() -> Iterator[Tuple[System, ROSProxy]]:
    rsw = ROSWire()
    with rsw.launch('brass') as sut:
        with sut.roscore() as ros:
            time.sleep(5)
            yield (sut, ros)


@contextlib.contextmanager
def build_shell_proxy() -> Iterator[ShellProxy]:
    rsw = ROSWire()
    image = 'brass'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.shell


@contextlib.contextmanager
def build_file_proxy() -> Iterator[FileProxy]:
    rsw = ROSWire()
    image = 'brass'
    desc = SystemDescription(image, [], [], [])
    with rsw.launch(image, desc) as sut:
        yield sut.files


def test_parameters():
    with build_test_environment() as (sut, ros):
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
    with build_ardu() as (sut, ros):
        db_type = sut.description.types
        db_fmt = sut.description.formats
        cmd = ' '.join([
            "/ros_ws/src/ArduPilot/build/sitl/bin/arducopter",
            "--model copter"
        ])
        sut.shell.non_blocking_execute(cmd)

        ros.launch('mavros', 'apm.launch', fcu_url='tcp://127.0.0.1:5760@5760')
        time.sleep(10)


        print(list(ros.nodes))
        assert set(ros.nodes) == {'/mavros', '/rosout'}
        assert '/mavros' in ros.nodes
        assert '/rosout' in ros.nodes
        assert '/cool' not in ros.nodes

        # with ros.record('/tmp/baggio.bag') as recorder:
        #     time.sleep(30)

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

        # arm the copter
        req: Message = db_type.from_dict(db_fmt.messages['mavros_msgs/SetModeRequest'], {
            'base_mode': 64,
            'custom_mode': ''
        })
        res = ros.services['/mavros/set_mode'].call(req)
        assert res.success


if __name__ == '__main__':
    test_arducopter()
