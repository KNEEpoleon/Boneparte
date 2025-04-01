cmake_minimum_required(VERSION 3.22)
project(surgical_robot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable
add_executable(move_to_pose src/move_to_pose.cpp)

ament_target_dependencies(move_to_pose
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)

# Install launch folder
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the executable
install(TARGETS move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
