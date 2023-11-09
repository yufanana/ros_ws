import launch
from launch_ros.actions import Node

_PACKAGE_NAME = 'target_offset'

def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription([

        Node(
            package=_PACKAGE_NAME,
            executable='oc',
            name='target_detector'
            ),
        
        Node(
            package=_PACKAGE_NAME,
            executable='video',
            name='video'
            )
    ])
