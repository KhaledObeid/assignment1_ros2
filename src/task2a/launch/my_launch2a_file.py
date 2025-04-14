from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_cmd = Node(
        package='task2a',
        executable='node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('input_scan', '/scan_raw'), ('output_vel', '/nav_vel')]
    )
    ld = LaunchDescription()
    ld.add_action(node_cmd)
    return ld
