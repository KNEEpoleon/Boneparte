from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parasight_dir = get_package_share_directory('parasight')

    return LaunchDescription([
        # Include the vision node launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(parasight_dir, 'launch', 'rs_launch.py')
            )
        ),

        Node(
            package='parasight',  # Name of your ROS 2 package
            executable='host',  # Name of the Python script
            name='segment_host',
            output='screen',  # Show logs in the terminal
        ),
    ])
