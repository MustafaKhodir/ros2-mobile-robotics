cmake_minimum_required(VERSION 3.8)
project(ros2_mobile_robotics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# The charger node. The two standalone C++ programs in cpp/ need no ROS and
# are built on their own with g++.
add_executable(robot_stopper_node cpp/robot_stopper_node.cpp)
ament_target_dependencies(robot_stopper_node rclcpp geometry_msgs sensor_msgs nav_msgs)

install(TARGETS robot_stopper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
