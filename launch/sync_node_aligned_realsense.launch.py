from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='rs_launch.py',
            arguments=[
                'enable_rgbd', 'true',
                'enable_sync', 'true',
                'align_depth.enable', 'true',
                'enable_color', 'true',
                'enable_depth', 'true'
            ]
        ),
        Node(
            package='sync_node',
            namespace='sync_node',
            executable='sync_node',
            name='sync_node'
        )
    ])