# -*- coding: utf-8 -*-
import pytest

import time


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_read(sut):
    with sut.roscore() as ros:
        config = ros.roslaunch.read('pickplace_playground.launch', package='fetch_gazebo')
        assert config.params['/arm_controller/follow_joint_trajectory/type'].value == 'robot_controllers/FollowJointTrajectoryController'
        assert config.params['/gazebo/bellows_joint/position/i_clamp'].value == 0.0

        actual_node_names = {node.name for node in config.nodes}
        expected_node_names = {'prepare_robot',
                               'gazebo',
                               'crop_decimate',
                               'urdf_spawner',
                               'head_camera_nodelet_manager',
                               'gazebo_gui',
                               'robot_state_publisher',
                               'points_downsample',
                               'cmd_vel_mux'}
        assert actual_node_names == expected_node_names
