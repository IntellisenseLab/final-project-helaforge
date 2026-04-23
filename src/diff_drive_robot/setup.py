from setuptools import setup
import os
from glob import glob

package_name = 'diff_drive_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        # URDF / xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='QBot2 Semantic Robot Navigator with YOLO, SLAM, Nav2 and Voice Commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_navigator = diff_drive_robot.semantic_navigator:main',
            'voice_commander = diff_drive_robot.voice_commander:main',
            'arrow_teleop = diff_drive_robot.arrow_teleop:main',
            'odom_to_tf = diff_drive_robot.odom_to_tf:main',
            'yolo_tracker = diff_drive_robot.yolo_tracker:main',
            'kinect_bridge = diff_drive_robot.kinect_bridge:main',
            'kobuki_driver = diff_drive_robot.kobuki_driver:main',
        ],
    },
)
