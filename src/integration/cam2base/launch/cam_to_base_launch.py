import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node  # Correct import for ROS 2 Humble

def generate_launch_description():
    return LaunchDescription([
        # Launch the C++ node
        Node(
            package='cam2base',  # Name of your ROS 2 package
            executable='hand2eye_tf_publisher',  # Name of the C++ executable
            name='hand2eye_tf_publisher_cpp',
            output='screen',  # Show logs in the terminal
        ),
        
        # Launch the Python node
        Node(
            package='cam2base',  # Name of your ROS 2 package
            executable='drill_pose_publisher.py',  # Name of the Python script
            name='drill_pose_publisher_python',
            output='screen',  # Show logs in the terminal
        ),
        Node(
            package='cam2base',  # Name of your ROS 2 package
            executable='pointcloud_base_publisher.py',  # Name of the Python script
            name='pointcloud_base_publisher_python',
            output='screen',  # Show logs in the terminal
        ),
    ])
