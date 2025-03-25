cmake_minimum_required(VERSION 3.8)
project(kuka_control_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(moveit_control_node src/moveit_control_node.cpp)
ament_target_dependencies(moveit_control_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(moveit_test_node src/moveit_test.cpp)
ament_target_dependencies(moveit_test_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  moveit_control_node
  moveit_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()