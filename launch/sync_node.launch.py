from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bridge_py_s',
            executable='bridge_py_s_node',
            name='JAXA-cFS-Bridge',
            parameters="./config/params.yaml"
        ),
        Node(
            package='sync_node',
            namespace='sync_node',
            executable='sync_node',
            name='sync_node'
        )
    ])