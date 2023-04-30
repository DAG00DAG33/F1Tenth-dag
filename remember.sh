cd ros2_ws
source /opt/ros/foxy/setup.sh 
source install/setup.sh 

ros2 launch vesc_driver vesc_driver_node.launch.py

ros2 topic list -t
ros2 topic info /commands/motor/speed
ros2 interface show std_msgs/msg/Float64

ros2 topic pub --once /commands/motor/speed std_msgs/msg/Float64 "{data: 3000}"


ros2 run joy joy_node

ros2 topic echo /joy

ros2 run joystick_control joystick_control_node


# https://github.com/linorobot/ldlidar
ros2 launch ldlidar ldlidar.launch.py
ros2 launch ldlidar ldlidar.launch.py serial_port:=/dev/ttyUSB0
ros2 run tf2_ros static_transform_publisher 0 0 0 0.0 0.0 0.0 map laser


colcon build --symlink-install --parallel-workers 1   --packages-select joystick_control




export CMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake

