from typing import Iterator
import contextlib
import logging
import time

import pytest

from rozzy import Rozzy, ROSProxy


@contextlib.contextmanager
def build_test_environment() -> Iterator[ROSProxy]:
    rozzy = Rozzy()
    with rozzy.launch() as container:
        with container.roscore() as ros:
            time.sleep(5)
            yield ros


def test_parameters():
    with build_test_environment() as ros:
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


def main():
    logging.basicConfig()
    with build_test_environment() as ros:
        # TODO connect to simulator: allow visualisation
        # launch SITL on 5760
        # sim_vehicle.py -C -v ArduCopter --daemon --no-rebuild
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


        print(ros.topic_to_type)

        # launch_mavros = ros.launch("mavros apm.launch",
        #                            {'fcu_url': 'tcp://127.0.0.1:5760:5760'})
