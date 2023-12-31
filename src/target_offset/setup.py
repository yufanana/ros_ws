import os
from glob import glob
from setuptools import setup

package_name = 'target_offset'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('lib/python3.10/site-packages', package_name, 'weights'), glob(os.path.join('target_offset/weights', '*.pt')))
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
            'oc = target_offset.oc:main',
            'video = target_offset.video_sub:main',
            'oc_live = target_offset.oc_live_stream:main'
        ],
    },
)
