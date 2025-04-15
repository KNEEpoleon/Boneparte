import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Generates the launch description to start segmentation and camera calibration launch files.
    """
    ld = LaunchDescription()

    # 1. Include the launch file from the 'parasight' package
    parasight_segment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ # Assuming it's a Python launch file
            PathJoinSubstitution([
                FindPackageShare('parasight'),   # Find the 'parasight' package
                'launch',                        # Look inside its 'launch' directory
                'segment_rs_launch.py'           # Specify the launch file
            ])
        ])
        # Add launch_arguments here if segment_rs_launch.py requires any
        # launch_arguments={'arg_name': 'value'}.items()
    )

    # 2. Include the launch file from the 'cam2base' package
    cam2base_calib_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ # Assuming it's a Python launch file
            PathJoinSubstitution([
                FindPackageShare('cam2base'),    # Find the 'cam2base' package
                'launch',                        # Look inside its 'launch' directory
                'cam_to_base_launch.py'          # Specify the launch file
            ])
        ])
        # Add launch_arguments here if cam_to_base_launch.py requires any
        # launch_arguments={'arg_name': 'value'}.items()
    )

    # Add both IncludeLaunchDescription actions to the LaunchDescription
    ld.add_action(parasight_segment_launch)
    ld.add_action(cam2base_calib_launch)

    return ld

