#!/usr/bin/env python3
"""
Launch file for AVP TCP Server

Launches the avp_tcp_server node which streams drill poses
to Apple Vision Pro via TCP on port 5001.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tcp_server_pkg',
            executable='avp_tcp_server',
            name='avp_tcp_server',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])

