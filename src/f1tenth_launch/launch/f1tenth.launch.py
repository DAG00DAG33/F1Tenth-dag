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
        'config',
        'vesc.yaml'
    )

    sensors_config_default = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'sensors.yaml'
    )

    vesc_config_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config_default,
        description="Config file for all the nodes"
    )

    sensors_config_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config_default,
        description="Config file for all the nodes"
    )

    talker_ena_la = DeclareLaunchArgument(
        'talker_node_ena',
        default_value='0'
    )

    manual_control_ena_la = DeclareLaunchArgument(
        'manual_control_ena',
        default_value='1'
    )

    hysteresis_control_ena_la = DeclareLaunchArgument(
        'hysteresis_control_ena',
        default_value='1'
    )

    return LaunchDescription([
        vesc_config_la,
        sensors_config_la,
        talker_ena_la,
        manual_control_ena_la,
        hysteresis_control_ena_la,
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            condition=IfCondition(LaunchConfiguration('talker_node_ena'))
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        # Node(
        #    package='joystick_control',
        #    executable='joystick_control_node',
        #    name='joystick_control_node',
        #    condition=IfCondition(LaunchConfiguration('joystick_control_ena')),
        #    output='screen'
        # ),
        Node(
            package='manual_control',
            executable='manual_control_node',
            name='manual_control_node',
            condition=IfCondition(LaunchConfiguration('manual_control_ena')),
            output='screen'
        ),
        Node(
            package='hysteresis_control',
            executable='autonomous_control_node',
            name='hysteresis_control_node',
            condition=IfCondition(LaunchConfiguration('hysteresis_control_ena')),
            output='screen'
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
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[LaunchConfiguration('vesc_config')]
        ),
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "-1.5708", "0", "0", "base_link", "laser"]
        ),
  ])
