# -*- coding: utf-8 -*-
import pytest

import time

import dockerblade

from roswire.common import SystemState


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_state(sut):
    with sut.ros1() as ros:
        expected_pubs = {
            '/rosout_agg': ['/rosout']
        }
        expected_subs = {
            '/rosout': ['/rosout']
        }
        expected_services = {
            '/rosout/set_logger_level': ['/rosout'],
            '/rosout/get_loggers': ['/rosout']
        }
        expected = SystemState(publishers=expected_pubs,
                               subscribers=expected_subs,
                               services=expected_services)
        assert ros.state == expected


@pytest.mark.skip(reason="skipping ROS2 tests")
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_publishers(sut):
    sut.ros2.launch('simple.launch.py', package='launch')
    actual_state = sut.ros2.state
    actual_publishers = set(actual_state.publishers)
    expected_publishers = set({'/constraint_list',
                                '/map',
                                '/parameter_events',
                                '/rosout',
                                '/scan_matched_points2',
                                '/submap_list',
                                '/tf',
                                '/trajectory_node_list',
                                '/landmark_poses_list'})
    assert actual_publishers == expected_publishers


@pytest.mark.skip(reason="skipping ROS2 tests")
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_subscribers(sut):
    sut.ros2.launch('simple.launch.py', package='launch')
    actual_state = sut.ros2.state
    actual_subscribers = set(actual_state.subscribers)
    print(actual_subscribers)
    expected_subscribers = set({'/odom',
                                '/scan',
                                '/parameter_events',
                                '/submap_list',
                                '/imu',
                                ''})
    assert actual_subscribers == expected_subscribers


@pytest.mark.skip(reason="skipping ROS2 tests")
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_services(sut):
    sut.ros2.launch('simple.launch.py', package='launch')
    actual_state = sut.ros2.state
    actual_services = set(actual_state.services)
    print(actual_services)
    expected_services = set({'/cartographer_node/describe_parameters',
                             '/cartographer_node/get_parameter_types',
                             '/cartographer_node/get_parameters',
                             '/cartographer_node/list_parameters',
                             '/cartographer_node/set_parameters',
                             '/cartographer_node/set_parameters_atomically',
                             '/finish_trajectory',
                             '/launch_ros/describe_parameters',
                             '/launch_ros/get_parameter_types',
                             '/launch_ros/get_parameters',
                             '/launch_ros/list_parameters',
                             '/launch_ros/set_parameters',
                             '/launch_ros/set_parameters_atomically',
                             '/occupancy_grid_node/describe_parameters',
                             '/occupancy_grid_node/get_parameter_types',
                             '/occupancy_grid_node/get_parameters',
                             '/occupancy_grid_node/list_parameters',
                             '/occupancy_grid_node/set_parameters',
                             '/occupancy_grid_node/set_parameters_atomically',
                             '/start_trajectory',
                             '/submap_query',
                             '/write_state'})
    assert actual_services == expected_services

