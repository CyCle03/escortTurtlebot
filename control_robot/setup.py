from setuptools import setup
import os
from glob import glob

package_name = 'team_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일을 인식시키기 위한 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 명령어이름 = 패키지폴더명.파일명:메인함수이름
            'leader_node = team_project.leader_node:main',
            'follower_node = team_project.follower_node:main',
            'gest = team_project.gesture_turtle:main',
            
        ],
    },
)