cmake_minimum_required(VERSION 3.8)
project(cam2base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Python dependencies
find_package(rclpy REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(hand2eye_tf_publisher src/hand2eye_tf_publisher.cpp)
ament_target_dependencies(hand2eye_tf_publisher
    rclcpp
    tf2
    tf2_ros
    geometry_msgs    
)

# Make the Python script executable and install it
install(PROGRAMS
  src/drill_pose_publisher.py
  DESTINATION lib/cam2base
)

install(PROGRAMS
  src/pointcloud_base_publisher.py
  DESTINATION lib/cam2base
)

install(PROGRAMS
  src/pointcloud_shell_base_publisher.py
  DESTINATION lib/cam2base
)

install(DIRECTORY launch/
  DESTINATION share/cam2base/launch
)

install(TARGETS hand2eye_tf_publisher
    DESTINATION lib/cam2base)

ament_package()