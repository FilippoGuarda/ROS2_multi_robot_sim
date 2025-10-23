cmake_minimum_required(VERSION 3.8)
project(SMrTa_messages)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPosition.msg"
  "msg/FleetRobotPositions.msg"
  "msg/Task.msg"
  "msg/FleetTasks.msg"
  "msg/Assignment.msg"
  "msg/FleetAssignments.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_python_install_module(${PROJECT_NAME})

ament_package()