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
        ],
    },
    scripts=[
        'scripts/escort_vision_node',
        'scripts/escort_turtlebot_gui',
        'scripts/follower_detector_node',
        'scripts/leader_initial_move_node',
    ],
)
