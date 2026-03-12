from glob import glob

from setuptools import find_packages, setup

package_name = 'escort_turtlebot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/param', glob('param/*.yaml')),
        ('share/' + package_name + '/test', glob('test/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='penguin',
    maintainer_email='cherrybear03@naver.com',
    description='Launch and bridge utilities for TurtleBot escort follower experiments.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
            'pytest-ros',
            'launch',
            'launch_ros',
        ],
    },
    entry_points={
        'console_scripts': [
            'leader_initial_move_node = escort_turtlebot_pkg.leader_initial_move_node:main',
            'follower_detector_node = escort_turtlebot_pkg.follower_detector_node:main',
        ],
    },
)
