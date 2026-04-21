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
            glob('config/*')),
        # URDF / xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='QBot2 Semantic Robot Navigator with YOLO, SLAM, Nav2 and Voice Commands',
    license='MIT',
    scripts=[
        'scripts/odom_to_tf.py',
        'scripts/yolo_tracker.py',
        'scripts/sort.py',
        'scripts/semantic_navigator.py',
        'scripts/arrow_teleop.py',
        'scripts/voice_commander.py',
    ],
)
