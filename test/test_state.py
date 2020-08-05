# -*- coding: utf-8 -*-
import pytest

import time


@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_publishers(sut):
    actual_state = sut.ros2.state
    actual_publishers = {pub for pub in actual_state.publishers}
    expected_publishers = set({}) 
    assert actual_publishers == expected_publishers

@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_subscribers(sut):
    actual_state = sut.ros2.state
    actual_subscribers = {sub for sub in actual_state.subscribers}
    expected_subscribers = set({})
    assert actual_subscribers == expected_subscribers

@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_state_services(sut):
    actual_state = sut.ros2.state
    actual_services = {serv for serv in actual_state.services}
    expected_services = set({})
    assert actual_services == expected_services

