import launch
from launch_ros.actions import Node

_PACKAGE_NAME = 'visual_servo'


def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription([
        Node(
            package=_PACKAGE_NAME,
            executable='visual_servo',
            name='visual_servo',),

        Node(
            package=_PACKAGE_NAME,
            executable='waypoint_publisher',
            name='waypoint_publisher',)
    ])
