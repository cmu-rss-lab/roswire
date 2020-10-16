# -*- coding: utf-8 -*-
import pytest

import time


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_read(sut):
    with sut.roscore() as ros:
        config = ros.roslaunch.read('pickplace_playground.launch', package='fetch_gazebo')
        assert '/ros_ws/src/fetch_gazebo/fetch_gazebo/models:' in config.envs['GAZEBO_MODEL_PATH'].value

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


@pytest.mark.parametrize('sut', ['husky'], indirect=True)
def test_eval_args_in_launch_file(sut):
    with sut.roscore() as ros:
        config = ros.roslaunch.read('spawn_husky.launch', package='husky_gazebo')
        actual_node_names = {node.name for node in config.nodes}
        expected_node_names = {'base_controller_spawner',
                               'ekf_localization',
                               'robot_state_publisher',
                               'spawn_husky_model',
                               'twist_marker_server',
                               'twist_mux'}
        assert actual_node_names == expected_node_names


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_remappings(sut):
    with sut.roscore() as ros:
        launch = ros.roslaunch('pickplace_playground.launch',
                               package='fetch_gazebo',
                               node_to_remappings={'gazebo': [('/gazebo/model_states', '/funkybits')]})

        time.sleep(30)
        state = ros.state
        expected_nodes = {'/cmd_vel_mux',
                          '/gazebo',
                          '/head_camera/crop_decimate',
                          '/head_camera/depth_downsample/points_downsample',
                          '/head_camera/head_camera_nodelet_manager',
                          '/robot_state_publisher',
                          '/rosout'}
        assert state.nodes == expected_nodes
        published_topics = set(state.publishers)
        assert '/gazebo/model_states' not in state.publishers
        assert set(state.publishers['/funkybits']) == {'/gazebo'}


@pytest.mark.skip(reason='ROS2 is not fully supported')
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_locate_with_package_ros2(sut):
    actual_path = sut.ros2.launch.locate('cartographer.launch.py', package='turtlebot3_cartographer')
    expected_path = '/ros_ws/install/turtlebot3_cartographer/share/launch/cartographer.launch.py'
    assert actual_path == expected_path


@pytest.mark.skip(reason='ROS2 is not fully supported')
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_launch_ros2(sut):
    actual_controller_command = sut.ros2.launch.launch('cartographer.launch.py', package='turtlebot3_cartographer').popen.args
    expected_controller_command = 'ros2 launch turtlebot3_cartographer cartographer.launch.py'
    assert actual_controller_command == expected_controller_command


@pytest.mark.skip(reason='ROS2 is not fully supported')
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_launch_with_full_path(sut):
    actual_controller_command = sut.ros2.launch.launch('/ros_ws/src/turtlebot3/turtlebot3_cartographer/launch/cartographer.launch.py', package='turtlebot3_cartographer').popen.args
    expected_controller_command = 'ros2 launch turtlebot3_cartographer cartographer.launch.py'
    assert actual_controller_command == expected_controller_command
