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
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectPose.srv"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add executable
add_executable(move_to_pose src/move_to_pose.cpp)
add_executable(drill_motion_executor src/drill_motion_executor.cpp)

ament_target_dependencies(move_to_pose
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
)
target_link_libraries(drill_motion_executor "${cpp_typesupport_target}")
ament_target_dependencies(drill_motion_executor 
  geometry_msgs
  std_msgs
  moveit_ros_planning_interface
  rclcpp
)
target_include_directories(drill_motion_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
# Install all binaries
install(TARGETS move_to_pose drill_motion_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch folder
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
