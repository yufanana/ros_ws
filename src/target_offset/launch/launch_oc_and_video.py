from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='target_offset',
            namespace='',
            executable='oc',
            name='oc'
        ),
        Node(
            package='target_offset',
            namespace='',
            executable='video',
            name='video'
        )
    ])