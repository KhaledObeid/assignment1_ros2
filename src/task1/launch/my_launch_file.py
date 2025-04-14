#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create a Node action for our task1 node.
    node_cmd = Node(
        package='task1',           # Replace with your actual package name.
        executable='node',         # Replace with the target name you used in CMakeLists.txt.
        output='screen',
        parameters=[{'use_sim_time': True}],  # If using simulated time.
        remappings=[
            ('input_scan', '/scan_raw'),
            ('output_vel', '/nav_vel')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(node_cmd)
    return ld
