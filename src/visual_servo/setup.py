import os
from glob import glob
from setuptools import setup

# Main folder
package_name = 'visual_servo'

# Src folder
src = package_name + '/src'

# sub folders
statemachines = src + '/statemachines'
stateMachine = statemachines + '/stateMachine'

px4_interface = src + '/px4_interface'

controllers = src + '/controllers'

publishers = src + '/publishers'
# subscriptions = src + '/subscriptions'

camera = src + '/camera'
localization = camera + '/localization'
marker_detection = camera + '/marker_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, src, statemachines, stateMachine, px4_interface, controllers,
              camera, localization, marker_detection, publishers],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stream = visual_servo.camera_stream:main',
            'offboard = visual_servo.main:main',
            'target_pub = visual_servo.target_publisher:main',
            'waypoint_publisher = visual_servo.waypointPublisher:main'
        ],
    },
)

