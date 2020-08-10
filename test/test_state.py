# -*- coding: utf-8 -*-
import pytest

import time

import dockerblade

@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_publishers(sut):
    sut.ros2.launch_manager.launch('simple.launch.py', package='launch')
    actual_state = sut.ros2.state
    actual_publishers = {pub for pub in actual_state.publishers}
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

@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_subscribers(sut):
    actual_state = sut.ros2.state
    actual_subscribers = {sub for sub in actual_state.subscribers}
    print(actual_subscribers)
    expected_subscribers = set({})
    assert actual_subscribers == expected_subscribers

@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_services(sut):
    actual_state = sut.ros2.state
    actual_services = {serv for serv in actual_state.services}
    print(actual_services)
    expected_services = set({})
    assert actual_services == expected_services

