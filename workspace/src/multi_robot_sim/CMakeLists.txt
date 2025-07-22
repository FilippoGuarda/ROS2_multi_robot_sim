cmake_minimum_required(VERSION 3.5)
project(multi_robot_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)
include_directories(SYSTEM
  /usr/local/include #for lightsfm
)
# this needed to tell colcon to place launch files in the share folder
install(DIRECTORY launch worlds configs urdf
  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})


# add_executable(set_gazebo_state_node nodes/gazebo_set_state.cpp)
# ament_target_dependencies(set_gazebo_state_node rclcpp gazebo_msgs std_msgs)
# install( TARGETS
#   set_gazebo_state_node
#   DESTINATION lib/${PROJECT_NAME}
# )



ament_package()