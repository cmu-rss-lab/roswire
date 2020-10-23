# -*- coding: utf-8 -*-
import pytest

from roswire.common import MsgFormat, SrvFormat, Package, PackageDatabase


def test_to_and_from_dict():
    pkg = 'tf'
    msg_tf = MsgFormat.from_dict({
        'package': pkg,
        'name': 'tfMessage',
        'definition': 'geometry_msgs/TransformStamped[] transforms\n',
        'fields': [{'type': 'geometry_msgs/TransformStamped[]',
                    'name': 'transforms'}]})
    srv_fg = SrvFormat.from_dict({
        'package': pkg,
        'name': 'FrameGraph',
        'definition': '---\nstring dot_graph\n',
        'response': {
            'definition': 'string dot_graph\n',
            'fields': [{'type': 'string', 'name': 'dot_graph'}]}})
    p = Package(name=pkg,
                path='/ros_ws/src/geometry/tf',
                messages=[msg_tf],
                actions=[],
                services=[srv_fg])
    assert p == Package.from_dict(p.to_dict())


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_build(sut):
    path = '/opt/ros/melodic/share/tf'
    expected = Package.from_dict({
        'path': path,
        'name': 'tf',
        'messages': [
            {'name': 'tfMessage',
             'definition': 'geometry_msgs/TransformStamped[] transforms\n',
             'fields': [{'type': 'geometry_msgs/TransformStamped[]',
                         'name': 'transforms'}]}
        ],
        'services': [
            {'name': 'FrameGraph',
             'definition': '---\nstring dot_graph\n',
             'response': {
                'definition': 'string dot_graph',
                'fields': [{'type': 'string',
                            'name': 'dot_graph'}]}}
        ]
    })
    actual = Package.build(path, sut)
    assert actual == expected


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_database_paths(sut):
    expected = {
        '/opt/ros/melodic/share/moveit_ros_occupancy_map_monitor',
        '/opt/ros/melodic/share/common_msgs',
        '/opt/ros/melodic/share/nodelet_core',
        '/opt/ros/melodic/share/ros_comm',
        '/opt/ros/melodic/share/bond_core',
        '/opt/ros/melodic/share/ros_base',
        '/opt/ros/melodic/share/ros_core',
        '/opt/ros/melodic/share/roscpp_core',
        '/opt/ros/melodic/share/ros',
        '/opt/ros/melodic/share/actionlib',
        '/opt/ros/melodic/share/actionlib_msgs',
        '/opt/ros/melodic/share/amcl',
        '/opt/ros/melodic/share/angles',
        '/opt/ros/melodic/share/base_local_planner',
        '/opt/ros/melodic/share/bond',
        '/opt/ros/melodic/share/bondcpp',
        '/opt/ros/melodic/share/bondpy',
        '/opt/ros/melodic/share/camera_calibration_parsers',
        '/opt/ros/melodic/share/camera_info_manager',
        '/opt/ros/melodic/share/catkin',
        '/opt/ros/melodic/share/class_loader',
        '/opt/ros/melodic/share/clear_costmap_recovery',
        '/opt/ros/melodic/share/cmake_modules',
        '/opt/ros/melodic/share/control_msgs',
        '/opt/ros/melodic/share/control_toolbox',
        '/opt/ros/melodic/share/costmap_2d',
        '/opt/ros/melodic/share/cpp_common',
        '/opt/ros/melodic/share/cv_bridge',
        '/opt/ros/melodic/share/depth_image_proc',
        '/opt/ros/melodic/share/diagnostic_msgs',
        '/opt/ros/melodic/share/diagnostic_updater',
        '/opt/ros/melodic/share/dynamic_reconfigure',
        '/opt/ros/melodic/share/eigen_conversions',
        '/opt/ros/melodic/share/eigen_stl_containers',
        '/opt/ros/melodic/share/eigenpy',
        '/ros_ws/src/fetch_ros/fetch_depth_layer',
        '/ros_ws/src/fetch_ros/fetch_description',
        '/ros_ws/src/fetch_gazebo/fetch_gazebo',
        '/ros_ws/src/fetch_gazebo/fetch_gazebo_demo',
        '/ros_ws/src/fetch_ros/fetch_ikfast_plugin',
        '/ros_ws/src/fetch_ros/fetch_maps',
        '/ros_ws/src/fetch_ros/fetch_moveit_config',
        '/ros_ws/src/fetch_ros/fetch_navigation',
        '/opt/ros/melodic/share/gazebo_dev',
        '/opt/ros/melodic/share/gazebo_msgs',
        '/opt/ros/melodic/share/gazebo_plugins',
        '/opt/ros/melodic/share/gazebo_ros',
        '/opt/ros/melodic/share/gencpp',
        '/opt/ros/melodic/share/geneus',
        '/opt/ros/melodic/share/genlisp',
        '/opt/ros/melodic/share/genmsg',
        '/opt/ros/melodic/share/gennodejs',
        '/opt/ros/melodic/share/genpy',
        '/opt/ros/melodic/share/geometric_shapes',
        '/opt/ros/melodic/share/geometry_msgs',
        '/opt/ros/melodic/share/grasping_msgs',
        '/opt/ros/melodic/share/image_geometry',
        '/opt/ros/melodic/share/image_proc',
        '/opt/ros/melodic/share/image_transport',
        '/opt/ros/melodic/share/interactive_markers',
        '/opt/ros/melodic/share/joint_state_publisher',
        '/opt/ros/melodic/share/kdl_conversions',
        '/opt/ros/melodic/share/kdl_parser',
        '/opt/ros/melodic/share/laser_geometry',
        '/opt/ros/melodic/share/map_msgs',
        '/opt/ros/melodic/share/map_server',
        '/opt/ros/melodic/share/media_export',
        '/opt/ros/melodic/share/message_filters',
        '/opt/ros/melodic/share/message_generation',
        '/opt/ros/melodic/share/message_runtime',
        '/opt/ros/melodic/share/mk',
        '/opt/ros/melodic/share/move_base',
        '/opt/ros/melodic/share/move_base_msgs',
        '/opt/ros/melodic/share/moveit_commander',
        '/opt/ros/melodic/share/moveit_core',
        '/opt/ros/melodic/share/moveit_fake_controller_manager',
        '/opt/ros/melodic/share/moveit_kinematics',
        '/opt/ros/melodic/share/moveit_msgs',
        '/opt/ros/melodic/share/moveit_planners_ompl',
        '/opt/ros/melodic/share/moveit_python',
        '/opt/ros/melodic/share/moveit_ros_manipulation',
        '/opt/ros/melodic/share/moveit_ros_move_group',
        '/opt/ros/melodic/share/moveit_ros_perception',
        '/opt/ros/melodic/share/moveit_ros_planning',
        '/opt/ros/melodic/share/moveit_ros_planning_interface',
        '/opt/ros/melodic/share/moveit_ros_robot_interaction',
        '/opt/ros/melodic/share/moveit_ros_visualization',
        '/opt/ros/melodic/share/moveit_ros_warehouse',
        '/opt/ros/melodic/share/moveit_simple_controller_manager',
        '/opt/ros/melodic/share/nav_core',
        '/opt/ros/melodic/share/nav_msgs',
        '/opt/ros/melodic/share/navfn',
        '/opt/ros/melodic/share/nodelet',
        '/opt/ros/melodic/share/nodelet_topic_tools',
        '/opt/ros/melodic/share/object_recognition_msgs',
        '/opt/ros/melodic/share/octomap',
        '/opt/ros/melodic/share/octomap_msgs',
        '/opt/ros/melodic/share/ompl',
        '/opt/ros/melodic/share/open_karto',
        '/opt/ros/melodic/share/orocos_kdl',
        '/opt/ros/melodic/share/pcl_conversions',
        '/opt/ros/melodic/share/pcl_msgs',
        '/opt/ros/melodic/share/pcl_ros',
        '/opt/ros/melodic/share/pluginlib',
        '/opt/ros/melodic/share/polled_camera',
        '/opt/ros/melodic/share/python_orocos_kdl',
        '/opt/ros/melodic/share/python_qt_binding',
        '/opt/ros/melodic/share/random_numbers',
        '/opt/ros/melodic/share/realtime_tools',
        '/opt/ros/melodic/share/resource_retriever',
        '/opt/ros/melodic/share/rgbd_launch',
        '/opt/ros/melodic/share/robot_controllers',
        '/opt/ros/melodic/share/robot_controllers_interface',
        '/opt/ros/melodic/share/robot_controllers_msgs',
        '/opt/ros/melodic/share/robot_state_publisher',
        '/opt/ros/melodic/share/ros_environment',
        '/opt/ros/melodic/share/rosbag',
        '/opt/ros/melodic/share/rosbag_migration_rule',
        '/opt/ros/melodic/share/rosbag_storage',
        '/opt/ros/melodic/share/rosbash',
        '/opt/ros/melodic/share/rosboost_cfg',
        '/opt/ros/melodic/share/rosbuild',
        '/opt/ros/melodic/share/rosclean',
        '/opt/ros/melodic/share/rosconsole',
        '/opt/ros/melodic/share/rosconsole_bridge',
        '/opt/ros/melodic/share/roscpp',
        '/opt/ros/melodic/share/roscpp_serialization',
        '/opt/ros/melodic/share/roscpp_traits',
        '/opt/ros/melodic/share/roscreate',
        '/opt/ros/melodic/share/rosgraph',
        '/opt/ros/melodic/share/rosgraph_msgs',
        '/opt/ros/melodic/share/roslang',
        '/opt/ros/melodic/share/roslaunch',
        '/opt/ros/melodic/share/roslib',
        '/opt/ros/melodic/share/roslisp',
        '/opt/ros/melodic/share/roslz4',
        '/opt/ros/melodic/share/rosmake',
        '/opt/ros/melodic/share/rosmaster',
        '/opt/ros/melodic/share/rosmsg',
        '/opt/ros/melodic/share/rosnode',
        '/opt/ros/melodic/share/rosout',
        '/opt/ros/melodic/share/rospack',
        '/opt/ros/melodic/share/rosparam',
        '/opt/ros/melodic/share/rospy',
        '/opt/ros/melodic/share/rosservice',
        '/opt/ros/melodic/share/rostest',
        '/opt/ros/melodic/share/rostime',
        '/opt/ros/melodic/share/rostopic',
        '/opt/ros/melodic/share/rosunit',
        '/opt/ros/melodic/share/roswtf',
        '/opt/ros/melodic/share/rotate_recovery',
        '/opt/ros/melodic/share/rviz',
        '/opt/ros/melodic/share/sensor_msgs',
        '/opt/ros/melodic/share/shape_msgs',
        '/opt/ros/melodic/share/simple_grasping',
        '/opt/ros/melodic/share/slam_karto',
        '/opt/ros/melodic/share/smclib',
        '/opt/ros/melodic/share/sparse_bundle_adjustment',
        '/opt/ros/melodic/share/srdfdom',
        '/opt/ros/melodic/share/std_msgs',
        '/opt/ros/melodic/share/std_srvs',
        '/opt/ros/melodic/share/stereo_msgs',
        '/opt/ros/melodic/share/teleop_twist_keyboard',
        '/opt/ros/melodic/share/tf',
        '/opt/ros/melodic/share/tf2',
        '/opt/ros/melodic/share/tf2_eigen',
        '/opt/ros/melodic/share/tf2_geometry_msgs',
        '/opt/ros/melodic/share/tf2_kdl',
        '/opt/ros/melodic/share/tf2_msgs',
        '/opt/ros/melodic/share/tf2_py',
        '/opt/ros/melodic/share/tf2_ros',
        '/opt/ros/melodic/share/tf_conversions',
        '/opt/ros/melodic/share/topic_tools',
        '/opt/ros/melodic/share/trajectory_msgs',
        '/opt/ros/melodic/share/urdf',
        '/opt/ros/melodic/share/urdfdom_py',
        '/opt/ros/melodic/share/visualization_msgs',
        '/opt/ros/melodic/share/voxel_grid',
        '/opt/ros/melodic/share/warehouse_ros',
        '/opt/ros/melodic/share/xacro',
        '/opt/ros/melodic/share/xmlrpcpp'
    }
    pdb = PackageDatabase.build(sut)
    actual = set(pdb.paths)
    assert actual == expected


@pytest.mark.parametrize('sut', ['fetch'], indirect=True)
def test_database_from_paths(sut):
    paths = [
        '/opt/ros/melodic/share/angles',
        '/opt/ros/melodic/share/tf2',
        '/opt/ros/melodic/share/tf2_msgs',
        '/opt/ros/melodic/share/tf2_py',
        '/opt/ros/melodic/share/tf2_ros'
    ]
    db = PackageDatabase.from_paths(sut, paths)
    assert len(db) == len(paths)
    assert set(db) == {'angles', 'tf2', 'tf2_msgs', 'tf2_py', 'tf2_ros'}


@pytest.mark.skip(reason='ROS2 is not fully supported')
@pytest.mark.parametrize('sut', ['turtlebot3-ros2'], indirect=True)
def test_package_location_ros2(sut):
    expected_paths = {'/ros_ws/install/pcl_conversions',
                      '/ros_ws/install/ament_pep257',
                      '/ros_ws/install/class_loader',
                      '/ros_ws/install/tf2',
                      '/ros_ws/install/rosidl_typesupport_introspection_c',
                      '/ros_ws/install/rviz2',
                      '/ros_ws/install/rmw_fastrtps_shared_cpp',
                      '/ros_ws/install/tf2_msgs',
                      '/ros_ws/install/rosidl_typesupport_opensplice_c',
                      '/ros_ws/install/rcl_logging_noop',
                      '/ros_ws/install/turtlebot3_teleop',
                      '/ros_ws/install/turtlebot3_fake_node',
                      '/ros_ws/install/turtlebot3_gazebo',
                      '/ros_ws/install/turtlebot3_simulations',
                      '/ros_ws/install/test_msgs',
                      '/ros_ws/install/dwb_plugins',
                      '/ros_ws/install/ament_copyright',
                      '/ros_ws/install/rclcpp_lifecycle',
                      '/ros_ws/install/rcl_action',
                      '/ros_ws/install/ament_cmake_export_libraries',
                      '/ros_ws/install/geometry_msgs',
                      '/ros_ws/install/rviz_common',
                      '/ros_ws/install/rosgraph_msgs',
                      '/ros_ws/install/rosidl_adapter',
                      '/ros_ws/install/rcutils',
                      '/ros_ws/install/nav2_voxel_grid',
                      '/ros_ws/install/rmw_fastrtps_cpp',
                      '/ros_ws/install/ament_lint',
                      '/ros_ws/install/test_interface_files',
                      '/ros_ws/install/ament_cmake_auto',
                      '/ros_ws/install/ament_cmake_uncrustify',
                      '/ros_ws/install/ament_cmake',
                      '/ros_ws/install/ament_index_cpp',
                      '/ros_ws/install/nav_msgs',
                      '/ros_ws/install/dwb_msgs',
                      '/ros_ws/install/rviz_rendering_tests',
                      '/ros_ws/install/libcurl_vendor',
                      '/ros_ws/install/rviz_ogre_vendor',
                      '/ros_ws/install/nav_2d_utils',
                      '/ros_ws/install/costmap_queue',
                      '/ros_ws/install/rcpputils',
                      '/ros_ws/install/map_msgs',
                      '/ros_ws/install/nav2_costmap_2d',
                      '/ros_ws/install/rcl_interfaces',
                      '/ros_ws/install/ament_cmake_flake8',
                      '/ros_ws/install/ament_cmake_xmllint',
                      '/ros_ws/install/ament_cmake_gtest',
                      '/ros_ws/install/rclcpp',
                      '/ros_ws/install/std_srvs',
                      '/ros_ws/install/rcl',
                      '/ros_ws/install/builtin_interfaces',
                      '/ros_ws/install/ament_lint_auto',
                      '/ros_ws/install/console_bridge_vendor',
                      '/ros_ws/install/tf2_ros',
                      '/ros_ws/install/sensor_msgs',
                      '/ros_ws/install/rmw_implementation',
                      '/ros_ws/install/visualization_msgs',
                      '/ros_ws/install/ament_cmake_target_dependencies',
                      '/ros_ws/install/unique_identifier_msgs',
                      '/ros_ws/install/ament_cmake_ros',
                      '/ros_ws/install/fastrtps_cmake_module',
                      '/ros_ws/install/turtlebot3_navigation2',
                      '/ros_ws/install/opensplice_cmake_module',
                      '/ros_ws/install/rcl_yaml_param_parser',
                      '/ros_ws/install/libyaml_vendor',
                      '/ros_ws/install/urdf',
                      '/ros_ws/install/ament_lint_cmake',
                      '/ros_ws/install/ament_cpplint',
                      '/ros_ws/install/nav2_util',
                      '/ros_ws/install/ament_cmake_cpplint',
                      '/ros_ws/install/nav2_map_server',
                      '/ros_ws/install/nav2_bt_navigator',
                      '/ros_ws/install/python_cmake_module',
                      '/ros_ws/install/nav2_bringup',
                      '/ros_ws/install/tf2_geometry_msgs',
                      '/ros_ws/install/dwb_core',
                      '/ros_ws/install/ament_package',
                      '/ros_ws/install/osrf_pycommon',
                      '/ros_ws/install/ament_cmake_pep257',
                      '/ros_ws/install/pluginlib',
                      '/ros_ws/install/action_msgs',
                      '/ros_ws/install/cartographer_ros_msgs',
                      '/ros_ws/install/message_filters',
                      '/ros_ws/install/turtlebot3_cartographer',
                      '/ros_ws/install/ament_flake8',
                      '/ros_ws/install/dwb_controller',
                      '/ros_ws/install/nav2_dwb_controller',
                      '/ros_ws/install/rmw',
                      '/ros_ws/install/rviz_assimp_vendor',
                      '/ros_ws/install/turtlebot3_msgs',
                      '/ros_ws/install/nav_2d_msgs',
                      '/ros_ws/install/rviz_default_plugins',
                      '/ros_ws/install/pcl_msgs',
                      '/ros_ws/install/rosidl_cmake',
                      '/opt/ros/dashing',
                      '/ros_ws/install/ament_cmake_gmock',
                      '/ros_ws/install/nav2_lifecycle_manager',
                      '/ros_ws/install/rosidl_typesupport_introspection_cpp',
                      '/ros_ws/install/ament_cmake_export_definitions',
                      '/ros_ws/install/lifecycle_msgs',
                      '/ros_ws/install/dwb_critics',
                      '/ros_ws/install/rviz_rendering',
                      '/ros_ws/install/rosidl_typesupport_interface',
                      '/ros_ws/install/ament_cmake_libraries',
                      '/ros_ws/install/ament_lint_common',
                      '/ros_ws/install/rosidl_typesupport_fastrtps_c',
                      '/ros_ws/install/ament_cmake_python',
                      '/ros_ws/install/behaviortree_cpp',
                      '/ros_ws/install/rosidl_typesupport_cpp',
                      '/ros_ws/install/launch_testing',
                      '/ros_ws/install/ament_cmake_copyright',
                      '/ros_ws/install/rclcpp_action',
                      '/ros_ws/install/rclpy',
                      '/ros_ws/install/ament_cmake_pytest',
                      '/ros_ws/install/dynamixel_sdk',
                      '/ros_ws/install/ament_cmake_export_include_directories',
                      '/ros_ws/install/std_msgs',
                      '/ros_ws/install/resource_retriever',
                      '/ros_ws/install/nav2_world_model',
                      '/ros_ws/install/nav2_rviz_plugins',
                      '/ros_ws/install/rosidl_parser',
                      '/ros_ws/install/turtlebot3_description',
                      '/ros_ws/install/nav2_common',
                      '/ros_ws/install/ament_cmake_cppcheck',
                      '/ros_ws/install/ament_cmake_core',
                      '/ros_ws/install/rosidl_typesupport_opensplice_cpp',
                      '/ros_ws/install/robot_state_publisher',
                      '/ros_ws/install/tf2_eigen',
                      '/ros_ws/install/nav2_recoveries',
                      '/ros_ws/install/rosidl_default_runtime',
                      '/ros_ws/install/uncrustify_vendor',
                      '/ros_ws/install/tf2_sensor_msgs',
                      '/ros_ws/install/ament_cmake_export_interfaces',
                      '/ros_ws/install/navigation2',
                      '/ros_ws/install/rosidl_typesupport_c',
                      '/ros_ws/install/laser_geometry',
                      '/ros_ws/install/rosidl_generator_py',
                      '/ros_ws/install/rosidl_generator_cpp',
                      '/ros_ws/install/ament_cmake_test',
                      '/ros_ws/install/rviz_visual_testing_framework',
                      '/ros_ws/install/angles',
                      '/ros_ws/install/launch_testing_ament_cmake',
                      '/ros_ws/install/ament_cppcheck',
                      '/ros_ws/install/cartographer_ros',
                      '/ros_ws/install/rosidl_generator_dds_idl',
                      '/ros_ws/install/turtlebot3_node',
                      '/ros_ws/install/rosidl_generator_c',
                      '/ros_ws/install/kdl_parser',
                      '/ros_ws/install/rcl_lifecycle',
                      '/ros_ws/install/turtlebot3_bringup',
                      '/ros_ws/install/launch_ros',
                      '/ros_ws/install/rosidl_typesupport_fastrtps_cpp',
                      '/ros_ws/install/nav2_msgs',
                      '/ros_ws/install/composition_interfaces',
                      '/ros_ws/install/ament_xmllint',
                      '/ros_ws/install/hls_lfcd_lds_driver',
                      '/ros_ws/install/eigen3_cmake_module',
                      '/ros_ws/install/ament_index_python',
                      '/ros_ws/install/ament_cmake_lint_cmake',
                      '/ros_ws/install/rmw_implementation_cmake',
                      '/ros_ws/install/rmw_opensplice_cpp',
                      '/ros_ws/install/turtlebot3',
                      '/ros_ws/install/yaml_cpp_vendor',
                      '/ros_ws/install/nav2_amcl',
                      '/ros_ws/install/ament_cmake_export_dependencies',
                      '/ros_ws/install/nav2_behavior_tree',
                      '/ros_ws/install/nav2_navfn_planner',
                      '/ros_ws/install/ament_uncrustify',
                      '/ros_ws/install/rosidl_default_generators',
                      '/ros_ws/install/ament_cmake_include_directories',
                      '/ros_ws/install/launch',
                      '/ros_ws/install/ament_cmake_export_link_flags'
                      }
    actual_paths = set(PackageDatabase.paths(sut))
    assert actual_paths == expected_paths


