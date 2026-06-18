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
        # lightweight web dashboard
        (os.path.join('share', package_name, 'web'),
            glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Kobuki+Kinect v1/libfreenect Semantic Robot Navigator: YOLO/BoT-SORT, RTAB-Map, Nav2, Voice Commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_navigator = diff_drive_robot.semantic_navigator:main',
            'voice_commander = diff_drive_robot.voice_commander:main',
            'arrow_teleop = diff_drive_robot.arrow_teleop:main',
            'yolo_tracker = diff_drive_robot.yolo_tracker:main',
            'kobuki_driver = diff_drive_robot.kobuki_driver:main',
            'kinect_topic_bridge = diff_drive_robot.kinect_topic_bridge:main',
            'kinect_v1_freenect_driver = diff_drive_robot.kinect_v1_freenect_driver:main',
            'depth_preview = diff_drive_robot.depth_preview:main',
            'kinect_depth_grid = diff_drive_robot.kinect_depth_grid:main',
            'slam_pose_publisher = diff_drive_robot.slam_pose_publisher:main',
            'qbot_navigation_server = diff_drive_robot.qbot_navigation_server:main',
            'qbot_controller = diff_drive_robot.qbot_controller:main',
            'web_dashboard_server = diff_drive_robot.web_dashboard_server:main',
        ],
    },
)
