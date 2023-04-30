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

    #all_config_default = os.path.join(
    #    get_package_share_directory(package_name),
    #    'config',
    #    package_name + '.yaml'
    #)
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
        default_value='1'
    )

    return LaunchDescription([
        vesc_config_la,
        sensors_config_la,
        talker_ena_la,
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
            #condition=enable_condition('launch_joy_node'),
            output='screen'
        ),
        Node(
            package='joystick_control',
            executable='joystick_control_node',
            name='joystick_control_node',
            #condition=enable_condition('launch_joystick_control_node'),
            output='screen'
        ),
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[LaunchConfiguration('sensors_config')]
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([os.path.join(
        #        get_package_share_directory('ldlidar'), 'launch'),
        #        '/ldlidar.launch.py']),
        #    launch_arguments={'serial_port': '/dev/ttyUSB0'}.items(),
        #),
        #IncludeLaunchDescription(
        #   PythonLaunchDescriptionSource([os.path.join(
        #        get_package_share_directory('vesc_driver'), 'launch'),
        #        '/vesc_driver_node.launch.py']),
        #    launch_arguments={'port': '/dev/pepepep'}.items(),
        #    #port: "/dev/ttyACM0"
        ##),
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

        #Node(
        #    package='ldlidar',
        #    executable='ldlidar.launch.py',
        #    #arguments=['serial_port:=$(arg serial_port)'],
        #    #condition=enable_condition('launch_ldlidar_node'),
        #    output='screen'
        #),
        #Node(
        #    package='vesc_driver',
        #    executable='vesc_driver_node.launch.py',
        #    #condition=enable_condition('launch_vesc_driver_node'),
        #    output='screen'
        #),
  ])
