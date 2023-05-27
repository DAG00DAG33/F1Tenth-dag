import launch
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package_name = 'f1tenth_launch'

    vesc_config_default = os.path.join(
        get_package_share_directory(package_name),
        'config/car',
        'vesc.yaml'
    )

    sensors_config_default = os.path.join(
        get_package_share_directory(package_name),
        'config/car',
        'sensors.yaml'
    )

    manual_control_config_default = os.path.join(
        get_package_share_directory(package_name),
        'config/control',
        'manual_control.yaml'
    )

    control_config_default = os.path.join(
        get_package_share_directory(package_name),
        'config/control',
        'autonomous_pure_pursuit.yaml'
    )

    vesc_config_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config_default,
        description=""
    )

    sensors_config_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config_default,
        description=""
    )

    manual_control_config_la = DeclareLaunchArgument(
        'manual_control_config',
        default_value=manual_control_config_default,
        description=""
    )

    control_config_la = DeclareLaunchArgument(
        'control_config',
        default_value=control_config_default,
        description=""
    )

    return LaunchDescription([
        vesc_config_la,
        sensors_config_la,
        manual_control_config_la,
        control_config_la,
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "-1.5708", "0", "0", "base_link", "laser"]
        ),
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[LaunchConfiguration('sensors_config')]
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),
        # Node(
        #     package='vesc_ackermann',
        #     executable='vesc_to_odom_node',
        #     name='vesc_to_odom_node',
        #     parameters=[LaunchConfiguration('vesc_config')]
        # ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='manual_control',
            executable='manual_control_node',
            name='manual_control_node',
            output='screen',
            parameters=[LaunchConfiguration('manual_control_config')],
        ),
        Node(
            package='path_manager',
            executable='path_manager_node',
            name='target',
            #namespace='target',
            parameters=[LaunchConfiguration('control_config')]
        ),
        Node(
            package='path_manager',
            executable='path_manager_node',
            name='path',
            #namespace='path',
            parameters=[LaunchConfiguration('control_config')]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox_node',
            output='screen',
            parameters=[
                LaunchConfiguration('control_config'),
                {'use_sim_time': False}
            ],
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[LaunchConfiguration('control_config')]
        ),
        Node(
            package='razor_imu_ros2',
            executable='razor_imu_ros2_exe',
            output='screen',
            parameters=[LaunchConfiguration('sensors_config')]
        ),
        Node(
            package='imu_to_odom',
            executable='imu_to_odom_node',
            output='screen',
            parameters=[LaunchConfiguration('sensors_config')]
        ),
  ])
