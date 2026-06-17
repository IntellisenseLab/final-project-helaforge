import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Config files ───────────────────────────────────────────────
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2_rviz.rviz')

    # ── Launch arguments ───────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    map_file = LaunchConfiguration('map', default='')
    autostart = LaunchConfiguration('autostart', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock',
    )
    declare_use_slam = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Use SLAM (true) or a pre-saved map (false)',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to a map YAML file (used when use_slam=false)',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Auto-start the Nav2 lifecycle nodes',
    )

    # ── RTAB-Map (RGB-D SLAM) ──────────────────────────────────────────
# ── RTAB-Map (RGB-D SLAM) ──────────────────────────────────────────
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --Grid/FromDepth true --Grid/Sensor 0',
            'frame_id': 'base_footprint',        # Keeps it mapped to root frame
            'odom_frame_id': 'odom',            # Keeps it mapped to root odom
            'map_frame_id': 'map',
            'namespace': '',                    # Forces the wrapper to push nodes to root namespace if supported
            'qos_map': '1',                     # Sets Transient Local QoS directly via launch script parameter
            'visual_odometry': 'false',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'approx_sync': 'true',
            'wait_imu_to_init': 'false',
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'rtabmapviz': 'false'
        }.items(),
        condition=IfCondition(use_slam),
    )

    # ── Map Server (only when use_slam=false) ──────────────────────
    map_server = Node(
        condition=UnlessCondition(use_slam),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── AMCL (only when NOT using SLAM) ────────────────────────────
    amcl = Node(
        condition=UnlessCondition(use_slam),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Nav2 Lifecycle Manager for map/localization ────────────────
    lifecycle_mgr_localization = Node(
        condition=UnlessCondition(use_slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ── Nav2 Core Nodes ────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # ── Nav2 Lifecycle Manager (delayed to let SLAM produce map) ────
    delayed_lifecycle_mgr = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'bond_timeout': 20.0,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ],
                }],
            ),
        ],
    )

    # ── RViz ───────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_slam,
        declare_map,
        declare_autostart,

        # Localization
        rtabmap,
        map_server,
        amcl,
        lifecycle_mgr_localization,

        # Navigation
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        delayed_lifecycle_mgr,

        # Visualization
        rviz,
    ])