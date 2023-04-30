from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    config = LaunchConfiguration('config_file')
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='f1tenth_dag.yaml',
        description='Path to the config file for F1Tenth-dag'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=IfCondition(LaunchConfiguration('launch_joy_node')),
        output='screen'
    )

    joystick_control_node = Node(
        package='joystick_control',
        executable='joystick_control_node',
        name='joystick_control_node',
        condition=IfCondition(LaunchConfiguration('launch_joystick_control_node')),
        output='screen'
    )

    ldlidar_launch = Node(
        package='ldlidar',
        executable='ldlidar.launch.py',
        arguments=['serial_port:=', LaunchConfiguration('serial_port')],
        condition=IfCondition(LaunchConfiguration('launch_ldlidar_node')),
        output='screen'
    )

    vesc_driver_launch = Node(
        package='vesc_driver',
        executable='vesc_driver_node.launch.py',
        condition=IfCondition(LaunchConfiguration('launch_vesc_driver_node')),
        output='screen'
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'map', 'laser'],
        condition=IfCondition(LaunchConfiguration('launch_static_transform_publisher_node')),
        output='screen'
    )

    return LaunchDescription([
        declare_config_file,
        joy_node,
        joystick_control_node,
        ldlidar_launch,
        vesc_driver_launch,
        static_transform_publisher
    ])

