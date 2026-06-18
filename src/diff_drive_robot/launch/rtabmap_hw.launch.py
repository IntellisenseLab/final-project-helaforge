"""
rtabmap_hw.launch.py
====================
Real-hardware launch for Kobuki + Kinect v1/libfreenect + RTAB-Map + Nav2.

Node startup order (with delays to ensure stable TF before RTAB-Map):
  t=0s   kobuki_driver       → /odom, TF odom→base_footprint
  t=0s   kinect_v1_freenect  -> Kinect v1 RGB-D topics
  t=0s   kinect_topic_bridge → normalized /camera/* topics
  t=0s   rgbd_sync           → synchronized /rgbd_image for RTAB-Map
  t=0s   robot_state_pub     → TF base_footprint→base_link
  t=0s   static TFs          → base_link→camera_link→camera optical frames
  t=0s   depthimage_to_scan  → /scan (for Nav2 costmaps)
  t=3s   rtabmap             → /map OccupancyGrid + localisation
  t=18s  Nav2 nodes          → navigation stack (after RTAB-Map stabilises)
  t=6s   semantic_navigator  → manual/voice command handler
  t=18s  voice_commander     → (optional, enable via arg)
  t=0s   rviz2               → (optional, enable via arg)

Launch arguments:
  serial_port   (default /dev/ttyUSB0)  — Kobuki USB serial port
  wheel_diameter/separation/ticks_per_rev — odometry calibration
  left/right_wheel_scale, swap/invert encoders — odometry diagnostics/correction
  use_rviz      (default true)          — open RViz for visualisation
  use_rtabmap_viz (default true)        — open RTAB-Map visualisation
  use_voice     (default true)          — start voice_commander node
  use_live_depth_grid (default true)    — publish live /kinect_depth_map grid
  start_camera_driver (default true) — launch selected RGB-D driver
  camera_backend (default freenect)  — freenect, kinect_ros2 or openni2
  camera_driver_package (default openni2_camera) — only used by openni2 backend
  camera_driver_executable (default openni2_camera_driver)
  use_kinect_topic_bridge (default true) — normalize wrapper topics
  database_path (default ~/.ros/rtabmap.db) — RTAB-Map map database
  camera_x/y/z/roll/pitch/yaw              — Kinect mount pose on robot

Camera TF offsets:
  base_link → camera_link:
    x = 0.0 m, y = 0.0 m, z = 0.24 m
    roll = 0, pitch = 0, yaw = 0
    pitch is positive when the Kinect is tilted downward.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_robot')

    # ── Config file paths ──────────────────────────────────────────────────────
    nav2_params_file   = os.path.join(pkg_dir, 'config', 'nav2_params_hw.yaml')
    rtabmap_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    rviz_config_file   = os.path.join(pkg_dir, 'config', 'nav2_rviz.rviz')

    # ── Launch arguments ───────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Kobuki USB serial port')

    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter', default_value='0.070',
        description='Kobuki wheel diameter in metres')
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.230',
        description='Distance between Kobuki wheels in metres')
    ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev', default_value='2578.33',
        description='Encoder ticks per wheel revolution')
    left_wheel_scale_arg = DeclareLaunchArgument(
        'left_wheel_scale', default_value='1.0',
        description='Multiplier applied to left encoder distance')
    right_wheel_scale_arg = DeclareLaunchArgument(
        'right_wheel_scale', default_value='1.0',
        description='Multiplier applied to right encoder distance')
    swap_encoders_arg = DeclareLaunchArgument(
        'swap_encoders', default_value='false',
        description='Swap left/right encoder readings before odometry')
    invert_left_encoder_arg = DeclareLaunchArgument(
        'invert_left_encoder', default_value='false',
        description='Invert left encoder delta sign')
    invert_right_encoder_arg = DeclareLaunchArgument(
        'invert_right_encoder', default_value='false',
        description='Invert right encoder delta sign')
    straight_correction_enabled_arg = DeclareLaunchArgument(
        'straight_correction_enabled', default_value='true',
        description='Force small odom yaw drift to zero when commanded straight')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz visualisation')

    use_voice_arg = DeclareLaunchArgument(
        'use_voice', default_value='true',
        description='Launch voice_commander node')

    use_live_depth_grid_arg = DeclareLaunchArgument(
        'use_live_depth_grid', default_value='true',
        description='Publish live Kinect depth OccupancyGrid on /kinect_depth_map')

    use_rtabmap_viz_arg = DeclareLaunchArgument(
        'use_rtabmap_viz', default_value='true',
        description='Launch rtabmap_viz visualisation')

    start_camera_driver_arg = DeclareLaunchArgument(
        'start_camera_driver', default_value='true',
        description='Launch the RGB-D camera driver')

    camera_backend_arg = DeclareLaunchArgument(
        'camera_backend', default_value='freenect',
        description='RGB-D backend: freenect, kinect_ros2, or openni2')

    camera_driver_package_arg = DeclareLaunchArgument(
        'camera_driver_package', default_value='openni2_camera',
        description='ROS 2 package name for the openni2 backend')

    camera_driver_executable_arg = DeclareLaunchArgument(
        'camera_driver_executable', default_value='openni2_camera_driver',
        description='Executable name for the RGB-D camera driver')

    use_kinect_topic_bridge_arg = DeclareLaunchArgument(
        'use_kinect_topic_bridge', default_value='true',
        description='Normalize external Kinect wrapper topics to /camera/*')

    kinect_rgb_topic_arg = DeclareLaunchArgument(
        'kinect_rgb_topic', default_value='/camera/rgb/image_raw',
        description='RGB topic from the Kinect RGB-D driver')

    kinect_depth_topic_arg = DeclareLaunchArgument(
        'kinect_depth_topic', default_value='/camera/depth_registered/image_raw',
        description='Registered depth topic from the Kinect RGB-D driver')

    kinect_info_topic_arg = DeclareLaunchArgument(
        'kinect_info_topic', default_value='/camera/rgb/camera_info',
        description='Registered camera_info topic from the Kinect RGB-D driver')

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model', default_value='yolo26n.pt',
        description='Ultralytics YOLO model path/name')

    tracker_cfg_arg = DeclareLaunchArgument(
        'tracker_cfg', default_value='botsort.yaml',
        description='Ultralytics tracker config, e.g. botsort.yaml or bytetrack.yaml')

    every_n_arg = DeclareLaunchArgument(
        'every_n', default_value='10',
        description='Run YOLO every N RGB frames')

    return_max_linear_arg = DeclareLaunchArgument(
        'return_max_linear', default_value='0.10',
        description='Smooth PID return-home max linear speed in m/s')

    return_max_angular_arg = DeclareLaunchArgument(
        'return_max_angular', default_value='0.35',
        description='Smooth PID return-home max angular speed in rad/s')

    return_strategy_arg = DeclareLaunchArgument(
        'return_strategy', default_value='nav2_waypoints',
        description='Return home strategy: nav2_waypoints or pid')

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='Path to RTAB-Map database file')

    camera_x_arg = DeclareLaunchArgument(
        'camera_x', default_value='0.0',
        description='Kinect mount x offset from base_link, metres')
    camera_y_arg = DeclareLaunchArgument(
        'camera_y', default_value='0.0',
        description='Kinect mount y offset from base_link, metres')
    camera_z_arg = DeclareLaunchArgument(
        'camera_z', default_value='0.24',
        description='Kinect mount height from base_link, metres')
    camera_roll_arg = DeclareLaunchArgument(
        'camera_roll', default_value='0.0',
        description='Kinect mount roll, radians')
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='0.0',
        description='Kinect mount pitch, radians; positive tilts sensor downward')
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw', default_value='0.0',
        description='Kinect mount yaw, radians')
    scan_height_arg = DeclareLaunchArgument(
        'scan_height', default_value='40',
        description='Number of depth image rows used for /scan')

    serial_port   = LaunchConfiguration('serial_port')
    wheel_diameter = LaunchConfiguration('wheel_diameter')
    wheel_separation = LaunchConfiguration('wheel_separation')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    left_wheel_scale = LaunchConfiguration('left_wheel_scale')
    right_wheel_scale = LaunchConfiguration('right_wheel_scale')
    swap_encoders = LaunchConfiguration('swap_encoders')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    straight_correction_enabled = LaunchConfiguration('straight_correction_enabled')
    use_rviz      = LaunchConfiguration('use_rviz')
    use_voice     = LaunchConfiguration('use_voice')
    use_live_depth_grid = LaunchConfiguration('use_live_depth_grid')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    start_camera_driver = LaunchConfiguration('start_camera_driver')
    camera_backend = LaunchConfiguration('camera_backend')
    camera_driver_package = LaunchConfiguration('camera_driver_package')
    camera_driver_executable = LaunchConfiguration('camera_driver_executable')
    use_kinect_topic_bridge = LaunchConfiguration('use_kinect_topic_bridge')
    kinect_rgb_topic = LaunchConfiguration('kinect_rgb_topic')
    kinect_depth_topic = LaunchConfiguration('kinect_depth_topic')
    kinect_info_topic = LaunchConfiguration('kinect_info_topic')
    yolo_model = LaunchConfiguration('yolo_model')
    tracker_cfg = LaunchConfiguration('tracker_cfg')
    every_n = LaunchConfiguration('every_n')
    return_max_linear = LaunchConfiguration('return_max_linear')
    return_max_angular = LaunchConfiguration('return_max_angular')
    return_strategy = LaunchConfiguration('return_strategy')
    database_path = LaunchConfiguration('database_path')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    scan_height = LaunchConfiguration('scan_height')

    # ═══════════════════════════════════════════════════════════════════════════
    #  HARDWARE DRIVERS  (start immediately)
    # ═══════════════════════════════════════════════════════════════════════════

    # Kobuki serial driver — publishes /odom + TF odom→base_footprint
    kobuki_driver = Node(
        package='diff_drive_robot',
        executable='kobuki_driver',
        name='kobuki_driver',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'wheel_diameter': ParameterValue(wheel_diameter, value_type=float),
            'wheel_separation': ParameterValue(
                wheel_separation, value_type=float),
            'ticks_per_rev': ParameterValue(ticks_per_rev, value_type=float),
            'left_wheel_scale': ParameterValue(
                left_wheel_scale, value_type=float),
            'right_wheel_scale': ParameterValue(
                right_wheel_scale, value_type=float),
            'swap_encoders': ParameterValue(swap_encoders, value_type=bool),
            'invert_left_encoder': ParameterValue(
                invert_left_encoder, value_type=bool),
            'invert_right_encoder': ParameterValue(
                invert_right_encoder, value_type=bool),
            'straight_correction_enabled': ParameterValue(
                straight_correction_enabled, value_type=bool),
        }],
    )

    freenect_enabled = PythonExpression([
        "'", start_camera_driver, "'.lower() == 'true' and '",
        camera_backend, "' == 'freenect'"
    ])
    openni2_enabled = PythonExpression([
        "'", start_camera_driver, "'.lower() == 'true' and '",
        camera_backend, "' == 'openni2'"
    ])
    kinect_ros2_enabled = PythonExpression([
        "'", start_camera_driver, "'.lower() == 'true' and '",
        camera_backend, "' == 'kinect_ros2'"
    ])

    # Kinect v1 / Xbox 360 backend. It owns USB/libfreenect and publishes
    # OpenNI-style topics under /camera for the bridge below.
    freenect_camera_driver = Node(
        package='diff_drive_robot',
        executable='kinect_v1_freenect_driver',
        name='kinect_v1_freenect_driver',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/rgb/image_raw',
            'depth_topic': '/camera/depth_registered/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
            'frame_id': 'camera_rgb_optical_frame',
            'depth_format': 'registered',
            'fps': 15.0,
        }],
        condition=IfCondition(freenect_enabled),
    )

    # Optional OpenNI2 backend for Asus Xtion/PrimeSense style devices. Most
    # Kinect v1 units need the freenect backend instead.
    openni2_camera_driver = Node(
        package=camera_driver_package,
        executable=camera_driver_executable,
        name='driver',
        namespace='camera',
        output='screen',
        parameters=[{
            'depth_registration': True,
            'use_device_time': True,
            'rgb_frame_id': 'camera_rgb_optical_frame',
            'depth_frame_id': 'camera_depth_optical_frame',
            'ir_frame_id': 'camera_ir_optical_frame',
        }],
        condition=IfCondition(openni2_enabled),
    )

    # Optional C++ Kinect Xbox 360 backend from matlabbe/kinect_ros2, as used
    # by RTAB-Map's Kinect Xbox 360 example. Install that package before using
    # camera_backend:=kinect_ros2.
    kinect_ros2_camera_driver = Node(
        package='kinect_ros2',
        executable='kinect_ros2_node',
        name='kinect_ros2',
        output='screen',
        parameters=[{
            'depth_registration': True,
            'rgb_frame_id': 'camera_rgb_optical_frame',
            'depth_frame_id': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('rgb/image_raw', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth_registered/image_raw',
             '/camera/depth_registered/image_raw'),
            ('depth_registered/camera_info', '/camera/rgb/camera_info'),
        ],
        condition=IfCondition(kinect_ros2_enabled),
    )

    kinect_topic_bridge = Node(
        package='diff_drive_robot',
        executable='kinect_topic_bridge',
        name='kinect_topic_bridge',
        output='screen',
        parameters=[{
            'source_rgb_topic': kinect_rgb_topic,
            'source_depth_topic': kinect_depth_topic,
            'source_camera_info_topic': kinect_info_topic,
            'target_rgb_topic': '/camera/image_raw',
            'target_depth_topic': '/camera/depth/image_raw',
            'target_camera_info_topic': '/camera/camera_info',
            'output_frame_id': 'camera_rgb_optical_frame',
        }],
        condition=IfCondition(use_kinect_topic_bridge),
    )

    # RTAB-Map examples commonly pre-sync RGB, registered depth and CameraInfo
    # into a single RGBDImage. This is more robust than letting every RTAB-Map
    # node approximate-sync three topics independently.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.10,
            'qos_image': 1,
            'qos_camera_info': 1,
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('rgbd_image', '/rgbd_image'),
        ],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    #  ROBOT STATE PUBLISHER  (minimal URDF for TF: base_footprint → base_link)
    # ═══════════════════════════════════════════════════════════════════════════
    # Minimal URDF string — just declares the base_footprint→base_link joint.
    # The actual robot geometry does not need to be modelled for real-HW SLAM.
    MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="kobuki_hw">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child  link="base_link"/>
    <origin xyz="0 0 0.001" rpy="0 0 0"/>
  </joint>
</robot>"""

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': MINIMAL_URDF,
            'use_sim_time': False,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    #  STATIC TF PUBLISHERS  (camera mounting on QBot2 standard design)
    # ═══════════════════════════════════════════════════════════════════════════
    # Kinect v1 mounting:
    #   base_link origin is the geometric centre of the robot at ground level.
    #   Camera is mounted 5 cm forward and 45 cm above base_link by default.
    #   camera_link → optical frames: REP-103 rotation (x→right, y→down, z→fwd)

    # base_link → camera_link
    # (update x/y/z here if your physical mounting differs)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        output='screen',
        arguments=[
            '--x', camera_x, '--y', camera_y, '--z', camera_z,
            '--roll', camera_roll, '--pitch', camera_pitch, '--yaw', camera_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # camera_link → camera_depth_optical_frame
    # REP-103: optical frame has Z forward, X right, Y down
    # Relative to camera_link (Z up, X forward): roll=-π/2, yaw=-π/2
    tf_camera_to_depth_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_to_depth_optical',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963',
            '--pitch', '0.0',
            '--yaw', '-1.5707963',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_optical_frame',
        ],
    )

    # camera_link → camera_rgb_optical_frame (same transform for registered images)
    tf_camera_to_rgb_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_to_rgb_optical',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5707963',
            '--pitch', '0.0',
            '--yaw', '-1.5707963',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_rgb_optical_frame',
        ],
    )

    # ═══════════════════════════════════════════════════════════════════════════
    #  DEPTH → LASER SCAN  (feeds Nav2 costmap obstacle layer)
    # ═══════════════════════════════════════════════════════════════════════════
    # Converts Kinect depth image to a 2D LaserScan on /scan.
    # This gives Nav2 real-time obstacle sensing for local costmap.
    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height':     ParameterValue(scan_height, value_type=int),
            'range_min':       0.50,        # Kinect v1 min reliable range
            'range_max':       3.50,
            'output_frame_id': 'camera_rgb_optical_frame',
            'use_sim_time':    False,
        }],
        remappings=[
            ('image',     '/camera/depth/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('scan',      '/scan'),
        ],
    )

    kinect_depth_grid = Node(
        package='diff_drive_robot',
        executable='kinect_depth_grid',
        name='kinect_depth_grid',
        output='screen',
        parameters=[{
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'map_topic': '/kinect_depth_map',
            'frame_id': 'base_footprint',
            'camera_height': ParameterValue(camera_z, value_type=float),
            'camera_pitch': ParameterValue(camera_pitch, value_type=float),
            'resolution': 0.07,
            'forward_min': 0.50,
            'forward_max': 3.50,
            'lateral_range': 2.50,
            'min_obstacle_height': 0.08,
            'max_obstacle_height': 1.40,
            'pixel_step': 6,
            'obstacle_min_points': 2,
            'raytrace_free_space': True,
        }],
        condition=IfCondition(use_live_depth_grid),
    )

    # ═══════════════════════════════════════════════════════════════════════════
    #  RTAB-MAP  (delayed 3 s — allow kobuki/kinect/TF to come up first)
    # ═══════════════════════════════════════════════════════════════════════════
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'database_path': database_path,
                'subscribe_rgbd': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'approx_sync': False,
            },
        ],
        remappings=[
            ('rgbd_image',      '/rgbd_image'),
            ('rgb/image',       '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('odom',            '/odom'),
            ('map',             '/map'),
            ('grid_map',        '/map'),        # → Nav2 static_layer
        ],
        arguments=['--delete_db_on_start'],     # REMOVE this flag to resume a map
    )

    delayed_rtabmap = TimerAction(period=3.0, actions=[rtabmap_node])

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            rtabmap_params_file,
            {
                'use_sim_time': False,
                'subscribe_rgbd': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'approx_sync': False,
            },
        ],
        remappings=[
            ('rgbd_image',      '/rgbd_image'),
            ('rgb/image',       '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image',     '/camera/depth/image_raw'),
            ('odom',            '/odom'),
        ],
        condition=IfCondition(use_rtabmap_viz),
    )

    # ═══════════════════════════════════════════════════════════════════════════
    #  NAV2 STACK  (delayed 18 s — wait for RTAB-Map to build initial map)
    # ═══════════════════════════════════════════════════════════════════════════
    nav2_params = {'use_sim_time': False}

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, nav2_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, nav2_params],
        remappings=[
            ('cmd_vel',          'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
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
    )

    delayed_nav2 = TimerAction(
        period=18.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            nav2_lifecycle_manager,
        ])

    # ═══════════════════════════════════════════════════════════════════════════
    #  APPLICATION NODES  (delayed 18 s — after Nav2 is ready)
    # ═══════════════════════════════════════════════════════════════════════════
    semantic_navigator = Node(
        package='diff_drive_robot',
        executable='semantic_navigator',
        name='semantic_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'tracker_cfg':  tracker_cfg,
            'yolo_model':   yolo_model,
            'every_n':      ParameterValue(every_n, value_type=int),
            'return_max_linear': ParameterValue(
                return_max_linear, value_type=float),
            'return_max_angular': ParameterValue(
                return_max_angular, value_type=float),
            'return_strategy': return_strategy,
        }],
    )

    delayed_semantic = TimerAction(period=6.0, actions=[semantic_navigator])

    voice_commander = Node(
        package='diff_drive_robot',
        executable='voice_commander',
        name='voice_commander',
        output='screen',
        condition=IfCondition(use_voice),
    )
    delayed_voice = TimerAction(period=18.0, actions=[voice_commander])

    # ═══════════════════════════════════════════════════════════════════════════
    #  RVIZ  (optional)
    # ═══════════════════════════════════════════════════════════════════════════
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    # ═══════════════════════════════════════════════════════════════════════════
    #  ASSEMBLE
    # ═══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        # Arguments
        serial_port_arg,
        wheel_diameter_arg,
        wheel_separation_arg,
        ticks_per_rev_arg,
        left_wheel_scale_arg,
        right_wheel_scale_arg,
        swap_encoders_arg,
        invert_left_encoder_arg,
        invert_right_encoder_arg,
        straight_correction_enabled_arg,
        use_rviz_arg,
        use_voice_arg,
        use_live_depth_grid_arg,
        use_rtabmap_viz_arg,
        start_camera_driver_arg,
        camera_backend_arg,
        camera_driver_package_arg,
        camera_driver_executable_arg,
        use_kinect_topic_bridge_arg,
        kinect_rgb_topic_arg,
        kinect_depth_topic_arg,
        kinect_info_topic_arg,
        yolo_model_arg,
        tracker_cfg_arg,
        every_n_arg,
        return_max_linear_arg,
        return_max_angular_arg,
        return_strategy_arg,
        database_path_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        scan_height_arg,

        # t = 0s — Hardware + TF
        kobuki_driver,
        freenect_camera_driver,
        openni2_camera_driver,
        kinect_ros2_camera_driver,
        kinect_topic_bridge,
        rgbd_sync,
        robot_state_publisher,
        tf_base_to_camera,
        tf_camera_to_depth_optical,
        tf_camera_to_rgb_optical,
        depth_to_laserscan,
        kinect_depth_grid,

        # t = 3s — RTAB-Map SLAM
        delayed_rtabmap,
        rtabmap_viz,

        # t = 18s — Nav2, t = 6s — SemanticNavigator command subscriber
        delayed_nav2,
        delayed_semantic,

        # Optional
        delayed_voice,
        rviz,
    ])
