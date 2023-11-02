import launch
from launch_ros.actions import Node

_PACKAGE_NAME = 'visual_servo'

def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription([
        Node(
            package=_PACKAGE_NAME,
            executable='offboard',
            name='offboard'
            ),

        Node(
            package='target_offset',
            executable='oc',
            name='target_detector'
            ),
        
        Node(
            package='target_offset',
            executable='video',
            name='video'
            )
    ])
