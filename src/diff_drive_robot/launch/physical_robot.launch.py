import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_robot')
    
    # ── URDF via xacro ─────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # ── Launch arguments ───────────────────────────────────────────────
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Kobuki base',
    )
    
    # ── Robot State Publisher ──────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': False,
        }],
    )
    
    # ── Joint State Publisher ──────────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    
    # ── Kobuki Driver ──────────────────────────────────────────────────
    kobuki_driver = Node(
        package='diff_drive_robot',
        executable='kobuki_driver',
        name='kobuki_driver',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
        }],
    )
    
    # ── Kinect Bridge ──────────────────────────────────────────────────
    kinect_bridge = Node(
        package='diff_drive_robot',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
    )
    
    return LaunchDescription([
        declare_serial_port,
        robot_state_publisher,
        joint_state_publisher,
        kobuki_driver,
        kinect_bridge,
    ])
