import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Launch lbr_bringup move_group.launch.py
    lbr_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'), # Find the package
                'launch',                       # Go into its launch directory
                'move_group.launch.py'          # Specify the launch file
            ])
        ]),
        # Pass launch arguments as a dictionary
        launch_arguments={
            'model': 'med7',
            'mode': 'hardware',
            'rviz': 'true'
        }.items()
    )

    # 2. Launch surgical_robot_planner drill_motion_executor.launch.py
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('surgical_robot_planner'),
                'launch',
                'drill_motion_executor.launch.py'
            ])
        ]),
        launch_arguments={'model': 'med7'}.items()
    )

    # 3. Run serialcomms write_to_serial node
    serial_node = Node(
        package='serialcomms',
        executable='write_to_serial',
        name='write_to_serial_node', # Define a specific node name
        output='screen' # Show node output directly in the terminal
    )

    # 4. Run the tcp_pkg node
    avp_ros_node = Node(
        package='tcp_server_pkg',  # Name of your ROS 2 package
        executable='tcp_server_node',  # Name of the Python script
        name='tcp_server_node',
        output='screen',  # Show logs in the terminal
    )
    # Add the actions to the launch description
    ld.add_action(lbr_bringup_launch)
    ld.add_action(planner_launch)
    ld.add_action(serial_node)
    ld.add_action(avp_ros_node)

    return ld