import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────────────
    pkg_dir = get_package_share_directory('diff_drive_robot')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # ── URDF via xacro ─────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # ── World file ─────────────────────────────────────────────────────
    # By default, use the local filename and prepend the package path
    world_filename = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([
        FindPackageShare('diff_drive_robot'),
        'worlds',
        world_filename
    ])

    # ── Launch arguments ───────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='room.sdf',
        description='Name of the world SDF file inside the worlds/ directory',
    )

    # ── Gazebo Harmonic (gz sim) ───────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # ── Robot State Publisher ──────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spawn the robot in Gazebo ──────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'qbot2_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
    )

    # ── ROS-Gazebo Bridge: DiffDrive (YAML config) ──────────────────────
    # YAML config maps model-scoped Gz DiffDrive topics to flat ROS topics
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── ROS-Gazebo Bridge: Camera (command-line args) ─────────────────
    # Sensor topics use different scoping — bridge by topic name matching
    gz_bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_scan',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    # ── Odom → TF broadcaster ─────────────────────────────────────────
    # Converts bridged /odom messages to odom → base_footprint TF
    odom_to_tf = Node(
        package='diff_drive_robot',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── native LaserScan from Gazebo is now used directly ─────────────

    # ── Build launch description ──────────────────────────────────────
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        gz_bridge_scan,
        odom_to_tf,
    ])

