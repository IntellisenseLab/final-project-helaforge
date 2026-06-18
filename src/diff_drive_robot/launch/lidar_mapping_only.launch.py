"""
LiDAR-only mapping test for real Kobuki hardware.

Starts only:
  - Kobuki serial odometry driver
  - LD19 LiDAR driver
  - base/laser TF
  - SLAM Toolbox
  - RViz focused on /scan and /map

No Kinect, no voice, no object detection, no A* navigation controller.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _first_device(patterns, fallback):
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return fallback


def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_robot')
    slam_params_file = os.path.join(
        pkg_dir, 'config', 'lidar_slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'lidar_mapping.rviz')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=_first_device([
            '/dev/serial/by-id/usb-Yujin_Robot_iClebo_Kobuki_kobuki_*-if00-port0',
            '/dev/serial/by-id/*Kobuki*',
        ], '/dev/ttyUSB0'),
        description='Kobuki serial port')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=_first_device([
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0',
            '/dev/serial/by-id/*CP210*',
        ], '/dev/ttyUSB1'),
        description='LD19 LiDAR serial port')
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='230400',
        description='LD19 LiDAR baud rate')
    laser_frame_arg = DeclareLaunchArgument(
        'laser_frame', default_value='laser_link',
        description='LiDAR frame')
    laser_x_arg = DeclareLaunchArgument(
        'laser_x', default_value='0.0',
        description='LiDAR x offset from base_link')
    laser_y_arg = DeclareLaunchArgument(
        'laser_y', default_value='0.0',
        description='LiDAR y offset from base_link')
    laser_z_arg = DeclareLaunchArgument(
        'laser_z', default_value='0.14',
        description='LiDAR height from base_link')
    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw', default_value='0.0',
        description='LiDAR yaw. Use 3.14159265 if mounted backward.')
    laser_scan_dir_arg = DeclareLaunchArgument(
        'laser_scan_dir', default_value='true',
        description='true=counterclockwise, false=clockwise')
    invert_left_encoder_arg = DeclareLaunchArgument(
        'invert_left_encoder', default_value='false',
        description='Invert left encoder direction')
    invert_right_encoder_arg = DeclareLaunchArgument(
        'invert_right_encoder', default_value='false',
        description='Invert right encoder direction')
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.230',
        description='Kobuki wheel separation in metres')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz')

    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    laser_scan_dir = LaunchConfiguration('laser_scan_dir')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    wheel_separation = LaunchConfiguration('wheel_separation')
    use_rviz = LaunchConfiguration('use_rviz')

    kobuki_driver = Node(
        package='diff_drive_robot',
        executable='kobuki_driver',
        name='kobuki_driver',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'wheel_separation': ParameterValue(wheel_separation, value_type=float),
            'invert_left_encoder': ParameterValue(
                invert_left_encoder, value_type=bool),
            'invert_right_encoder': ParameterValue(
                invert_right_encoder, value_type=bool),
            'straight_correction_enabled': True,
        }],
    )

    lidar_driver = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='lidar_driver',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': laser_frame,
            'port_name': lidar_port,
            'port_baudrate': ParameterValue(lidar_baudrate, value_type=int),
            'laser_scan_dir': ParameterValue(laser_scan_dir, value_type=bool),
        }],
    )

    minimal_urdf = """<?xml version="1.0"?>
<robot name="kobuki_lidar_mapping">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.001" rpy="0 0 0"/>
  </joint>
</robot>"""

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': minimal_urdf,
            'use_sim_time': False,
        }],
    )

    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_laser',
        output='screen',
        arguments=[
            '--x', laser_x, '--y', laser_y, '--z', laser_z,
            '--roll', '0.0', '--pitch', '0.0', '--yaw', laser_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', laser_frame,
        ],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': False,
                'mode': 'mapping',
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
            },
        ],
    )

    lifecycle_manager = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                    'bond_timeout': 0.0,
                }],
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        lidar_baudrate_arg,
        laser_frame_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_yaw_arg,
        laser_scan_dir_arg,
        invert_left_encoder_arg,
        invert_right_encoder_arg,
        wheel_separation_arg,
        use_rviz_arg,
        kobuki_driver,
        lidar_driver,
        robot_state_publisher,
        tf_base_to_laser,
        slam_node,
        lifecycle_manager,
        rviz,
    ])
