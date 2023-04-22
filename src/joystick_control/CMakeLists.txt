cmake_minimum_required(VERSION 3.5)
project(joystick_control)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()
#add_compile_options(-fpermissive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(joystick_control_node src/joystick_control_node.cpp)
ament_target_dependencies(joystick_control_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  joystick_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()