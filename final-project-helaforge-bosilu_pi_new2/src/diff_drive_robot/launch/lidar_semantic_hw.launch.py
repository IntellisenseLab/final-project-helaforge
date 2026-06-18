"""
lidar_semantic_hw.launch.py
===========================
Real-hardware stack for Kobuki + LD19/RPLidar-style 2D LiDAR mapping/navigation
with Kinect kept only for RGB-D object recognition.

This launch intentionally does not start RTAB-Map, Kinect depth grids, or
depthimage_to_laserscan. LiDAR owns /scan and SLAM Toolbox owns /map.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2_rviz.rviz')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=_first_device([
            '/dev/serial/by-id/usb-Yujin_Robot_iClebo_Kobuki_kobuki_*-if00-port0',
            '/dev/serial/by-id/*Kobuki*',
        ], '/dev/ttyUSB0'),
        description='Kobuki serial port')
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

    start_lidar_driver_arg = DeclareLaunchArgument(
        'start_lidar_driver', default_value='true',
        description='Start the LiDAR driver from this launch')
    lidar_driver_package_arg = DeclareLaunchArgument(
        'lidar_driver_package', default_value='ldlidar_stl_ros2',
        description='2D LiDAR driver package')
    lidar_driver_executable_arg = DeclareLaunchArgument(
        'lidar_driver_executable', default_value='ldlidar_stl_ros2_node',
        description='2D LiDAR driver executable')
    lidar_product_name_arg = DeclareLaunchArgument(
        'lidar_product_name', default_value='LDLiDAR_LD19',
        description='LDLiDAR product name')
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=_first_device([
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*-if00-port0',
            '/dev/serial/by-id/*CP210*',
        ], '/dev/ttyUSB1'),
        description='LiDAR serial port')
    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='230400',
        description='LiDAR serial baud rate')
    laser_scan_dir_arg = DeclareLaunchArgument(
        'laser_scan_dir', default_value='true',
        description='LDLiDAR scan direction: true=counterclockwise, false=clockwise')
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LaserScan topic for SLAM/navigation')
    laser_frame_arg = DeclareLaunchArgument(
        'laser_frame', default_value='laser_link',
        description='LiDAR frame id')
    laser_x_arg = DeclareLaunchArgument(
        'laser_x', default_value='0.0',
        description='LiDAR x offset from base_link, metres')
    laser_y_arg = DeclareLaunchArgument(
        'laser_y', default_value='0.0',
        description='LiDAR y offset from base_link, metres')
    laser_z_arg = DeclareLaunchArgument(
        'laser_z', default_value='0.14',
        description='LiDAR height from base_link, metres')
    laser_roll_arg = DeclareLaunchArgument(
        'laser_roll', default_value='0.0',
        description='LiDAR roll, radians')
    laser_pitch_arg = DeclareLaunchArgument(
        'laser_pitch', default_value='0.0',
        description='LiDAR pitch, radians')
    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw', default_value='0.0',
        description='LiDAR yaw, radians')

    start_kinect_driver_arg = DeclareLaunchArgument(
        'start_kinect_driver', default_value='true',
        description='Start Kinect driver for object recognition')
    kinect_device_index_arg = DeclareLaunchArgument(
        'kinect_device_index', default_value='0',
        description='libfreenect Kinect device index')
    camera_backend_arg = DeclareLaunchArgument(
        'camera_backend', default_value='freenect',
        description='Kinect backend: freenect or openni2')
    camera_driver_package_arg = DeclareLaunchArgument(
        'camera_driver_package', default_value='openni2_camera',
        description='OpenNI2 package if camera_backend:=openni2')
    camera_driver_executable_arg = DeclareLaunchArgument(
        'camera_driver_executable', default_value='openni2_camera_driver',
        description='OpenNI2 executable if camera_backend:=openni2')
    use_kinect_topic_bridge_arg = DeclareLaunchArgument(
        'use_kinect_topic_bridge', default_value='true',
        description='Normalize Kinect wrapper topics to /camera/*')
    kinect_rgb_topic_arg = DeclareLaunchArgument(
        'kinect_rgb_topic', default_value='/camera/rgb/image_raw',
        description='RGB topic from Kinect driver')
    kinect_depth_topic_arg = DeclareLaunchArgument(
        'kinect_depth_topic', default_value='/camera/depth_registered/image_raw',
        description='Registered depth topic from Kinect driver')
    kinect_info_topic_arg = DeclareLaunchArgument(
        'kinect_info_topic', default_value='/camera/rgb/camera_info',
        description='CameraInfo topic from Kinect driver')
    camera_x_arg = DeclareLaunchArgument(
        'camera_x', default_value='0.0',
        description='Kinect x offset from base_link, metres')
    camera_y_arg = DeclareLaunchArgument(
        'camera_y', default_value='0.0',
        description='Kinect y offset from base_link, metres')
    camera_z_arg = DeclareLaunchArgument(
        'camera_z', default_value='0.24',
        description='Kinect height from base_link, metres')
    camera_roll_arg = DeclareLaunchArgument(
        'camera_roll', default_value='0.0',
        description='Kinect roll, radians')
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='0.0',
        description='Kinect pitch, radians')
    camera_yaw_arg = DeclareLaunchArgument(
        'camera_yaw', default_value='0.0',
        description='Kinect yaw, radians')

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model', default_value='yolo26n_ncnn_model',
        description='Ultralytics YOLO NCNN model directory/path/name')
    tracker_cfg_arg = DeclareLaunchArgument(
        'tracker_cfg', default_value='botsort.yaml',
        description='Ultralytics tracker config')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='RGB image topic used by YOLO detection')
    yolo_imgsz_arg = DeclareLaunchArgument(
        'yolo_imgsz', default_value='640',
        description='YOLO inference image size')
    yolo_conf_arg = DeclareLaunchArgument(
        'yolo_conf', default_value='0.40',
        description='YOLO confidence threshold')
    detection_rate_slam_arg = DeclareLaunchArgument(
        'detection_rate_slam', default_value='3.0',
        description='YOLO inference rate while SLAM scan is active, Hz')
    detection_rate_navigation_arg = DeclareLaunchArgument(
        'detection_rate_navigation', default_value='5.0',
        description='YOLO inference rate during saved-map navigation, Hz')
    detection_enabled_arg = DeclareLaunchArgument(
        'detection_enabled', default_value='true',
        description='Enable/disable YOLO detection')
    preview_enabled_arg = DeclareLaunchArgument(
        'preview_enabled', default_value='false',
        description='Enable YOLO preview windows if a node supports them')
    publish_annotated_image_arg = DeclareLaunchArgument(
        'publish_annotated_image', default_value='false',
        description='Publish annotated YOLO images if a node supports them')
    save_video_arg = DeclareLaunchArgument(
        'save_video', default_value='false',
        description='Save YOLO debug video if a node supports it')
    every_n_arg = DeclareLaunchArgument(
        'every_n', default_value='10',
        description='Deprecated compatibility option; detection rate is now timer based')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz')
    use_voice_arg = DeclareLaunchArgument(
        'use_voice', default_value='false',
        description='Launch voice commander')
    use_semantic_arg = DeclareLaunchArgument(
        'use_semantic', default_value='true',
        description='Launch semantic object navigator')
    use_qbot_nav_arg = DeclareLaunchArgument(
        'use_qbot_nav', default_value='true',
        description='Launch copied A* navigation server and controller')
    use_rosbridge_arg = DeclareLaunchArgument(
        'use_rosbridge', default_value='false',
        description='Launch rosbridge websocket for the web UI')
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port', default_value='9090',
        description='Rosbridge websocket port')
    use_web_dashboard_arg = DeclareLaunchArgument(
        'use_web_dashboard', default_value='false',
        description='Serve the lightweight browser dashboard')
    web_dashboard_port_arg = DeclareLaunchArgument(
        'web_dashboard_port', default_value='8080',
        description='HTTP port for the browser dashboard')
    qbot_linear_speed_arg = DeclareLaunchArgument(
        'qbot_linear_speed', default_value='0.10',
        description='A* controller linear speed in m/s')
    qbot_max_angular_arg = DeclareLaunchArgument(
        'qbot_max_angular_speed', default_value='0.35',
        description='A* controller angular speed limit in rad/s')
    object_clearance_arg = DeclareLaunchArgument(
        'object_clearance', default_value='0.05',
        description='Desired front clearance from object in metres')
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius', default_value='0.20',
        description='Robot body radius used when converting object clearance to base goal')
    object_dedup_enabled_arg = DeclareLaunchArgument(
        'object_dedup_enabled', default_value='true',
        description='Merge repeated detections of the same object')
    object_dedup_distance_arg = DeclareLaunchArgument(
        'object_dedup_distance', default_value='0.30',
        description='Map-frame XY distance used to merge same object detections')
    object_dedup_same_class_only_arg = DeclareLaunchArgument(
        'object_dedup_same_class_only', default_value='true',
        description='Only merge object detections with the same YOLO class')
    object_dedup_update_position_arg = DeclareLaunchArgument(
        'object_dedup_update_position', default_value='true',
        description='Average duplicate detections into the stored object position')
    object_lidar_fusion_arg = DeclareLaunchArgument(
        'object_lidar_fusion', default_value='true',
        description='Use LiDAR range to refine Kinect object distance')
    object_lidar_window_arg = DeclareLaunchArgument(
        'object_lidar_window_deg', default_value='4.0',
        description='LiDAR bearing window around object centre, degrees')
    object_lidar_max_delta_arg = DeclareLaunchArgument(
        'object_lidar_max_delta', default_value='0.75',
        description='Maximum Kinect/LiDAR range disagreement accepted for fusion')

    serial_port = LaunchConfiguration('serial_port')
    wheel_diameter = LaunchConfiguration('wheel_diameter')
    wheel_separation = LaunchConfiguration('wheel_separation')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    left_wheel_scale = LaunchConfiguration('left_wheel_scale')
    right_wheel_scale = LaunchConfiguration('right_wheel_scale')
    swap_encoders = LaunchConfiguration('swap_encoders')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    straight_correction_enabled = LaunchConfiguration('straight_correction_enabled')

    start_lidar_driver = LaunchConfiguration('start_lidar_driver')
    lidar_driver_package = LaunchConfiguration('lidar_driver_package')
    lidar_driver_executable = LaunchConfiguration('lidar_driver_executable')
    lidar_product_name = LaunchConfiguration('lidar_product_name')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    laser_scan_dir = LaunchConfiguration('laser_scan_dir')
    scan_topic = LaunchConfiguration('scan_topic')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')

    start_kinect_driver = LaunchConfiguration('start_kinect_driver')
    kinect_device_index = LaunchConfiguration('kinect_device_index')
    camera_backend = LaunchConfiguration('camera_backend')
    camera_driver_package = LaunchConfiguration('camera_driver_package')
    camera_driver_executable = LaunchConfiguration('camera_driver_executable')
    use_kinect_topic_bridge = LaunchConfiguration('use_kinect_topic_bridge')
    kinect_rgb_topic = LaunchConfiguration('kinect_rgb_topic')
    kinect_depth_topic = LaunchConfiguration('kinect_depth_topic')
    kinect_info_topic = LaunchConfiguration('kinect_info_topic')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    yolo_model = LaunchConfiguration('yolo_model')
    tracker_cfg = LaunchConfiguration('tracker_cfg')
    image_topic = LaunchConfiguration('image_topic')
    yolo_imgsz = LaunchConfiguration('yolo_imgsz')
    yolo_conf = LaunchConfiguration('yolo_conf')
    detection_rate_slam = LaunchConfiguration('detection_rate_slam')
    detection_rate_navigation = LaunchConfiguration('detection_rate_navigation')
    detection_enabled = LaunchConfiguration('detection_enabled')
    preview_enabled = LaunchConfiguration('preview_enabled')
    publish_annotated_image = LaunchConfiguration('publish_annotated_image')
    save_video = LaunchConfiguration('save_video')
    every_n = LaunchConfiguration('every_n')
    use_rviz = LaunchConfiguration('use_rviz')
    use_voice = LaunchConfiguration('use_voice')
    use_semantic = LaunchConfiguration('use_semantic')
    use_qbot_nav = LaunchConfiguration('use_qbot_nav')
    use_rosbridge = LaunchConfiguration('use_rosbridge')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    use_web_dashboard = LaunchConfiguration('use_web_dashboard')
    web_dashboard_port = LaunchConfiguration('web_dashboard_port')
    qbot_linear_speed = LaunchConfiguration('qbot_linear_speed')
    qbot_max_angular_speed = LaunchConfiguration('qbot_max_angular_speed')
    object_clearance = LaunchConfiguration('object_clearance')
    robot_radius = LaunchConfiguration('robot_radius')
    object_dedup_enabled = LaunchConfiguration('object_dedup_enabled')
    object_dedup_distance = LaunchConfiguration('object_dedup_distance')
    object_dedup_same_class_only = LaunchConfiguration(
        'object_dedup_same_class_only')
    object_dedup_update_position = LaunchConfiguration(
        'object_dedup_update_position')
    object_lidar_fusion = LaunchConfiguration('object_lidar_fusion')
    object_lidar_window_deg = LaunchConfiguration('object_lidar_window_deg')
    object_lidar_max_delta = LaunchConfiguration('object_lidar_max_delta')

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

    lidar_driver = Node(
        package=lidar_driver_package,
        executable=lidar_driver_executable,
        name='lidar_driver',
        output='screen',
        parameters=[{
            'product_name': lidar_product_name,
            'topic_name': 'scan',
            'frame_id': laser_frame,
            'port_name': lidar_port,
            'port_baudrate': ParameterValue(lidar_baudrate, value_type=int),
            'laser_scan_dir': ParameterValue(laser_scan_dir, value_type=bool),
        }],
        remappings=[
            ('scan', scan_topic),
        ],
        condition=IfCondition(start_lidar_driver),
    )

    minimal_urdf = """<?xml version="1.0"?>
<robot name="kobuki_lidar_semantic">
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
            '--roll', laser_roll, '--pitch', laser_pitch, '--yaw', laser_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', laser_frame,
        ],
    )

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

    freenect_enabled = PythonExpression([
        "'", start_kinect_driver, "'.lower() == 'true' and '",
        camera_backend, "' == 'freenect'"
    ])
    openni2_enabled = PythonExpression([
        "'", start_kinect_driver, "'.lower() == 'true' and '",
        camera_backend, "' == 'openni2'"
    ])

    freenect_camera_driver = Node(
        package='diff_drive_robot',
        executable='kinect_v1_freenect_driver',
        name='kinect_v1_freenect_driver',
        output='screen',
        parameters=[{
            'device_index': ParameterValue(kinect_device_index, value_type=int),
            'rgb_topic': '/camera/rgb/image_raw',
            'depth_topic': '/camera/depth_registered/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
            'frame_id': 'camera_rgb_optical_frame',
            'depth_format': 'registered',
            'fps': 15.0,
        }],
        condition=IfCondition(freenect_enabled),
    )

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

    depth_preview = Node(
        package='diff_drive_robot',
        executable='depth_preview',
        name='depth_preview',
        output='screen',
        parameters=[{
            'depth_topic': '/camera/depth/image_raw',
            'preview_topic': '/camera/depth/preview',
        }],
        condition=IfCondition(start_kinect_driver),
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
                'scan_topic': scan_topic,
            },
        ],
    )

    slam_lifecycle_manager = TimerAction(
        period=2.0,
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

    slam_pose_publisher = Node(
        package='diff_drive_robot',
        executable='slam_pose_publisher',
        name='slam_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'map_frame': 'map',
            'base_frame': 'base_footprint',
        }],
        condition=IfCondition(use_qbot_nav),
    )

    navigation_server = Node(
        package='diff_drive_robot',
        executable='qbot_navigation_server',
        name='qbot_navigation_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'pose_topic': '/slam_pose',
            'map_topic': '/map',
            'waypoint_tolerance': 0.35,
            'inflation_radius': 3,
            'lookahead_distance': 0.6,
        }],
        condition=IfCondition(use_qbot_nav),
    )

    qbot_controller = Node(
        package='diff_drive_robot',
        executable='qbot_controller',
        name='qbot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'pose_topic': '/slam_pose',
            'goal_topic': '/ui_goal',
            'cmd_vel_topic': '/cmd_vel',
            'linear_speed': ParameterValue(qbot_linear_speed, value_type=float),
            'max_angular_speed': ParameterValue(
                qbot_max_angular_speed, value_type=float),
        }],
        condition=IfCondition(use_qbot_nav),
    )

    semantic_navigator = Node(
        package='diff_drive_robot',
        executable='semantic_navigator',
        name='semantic_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'tracker_cfg': tracker_cfg,
            'yolo_model': yolo_model,
            'image_topic': image_topic,
            'yolo_imgsz': ParameterValue(yolo_imgsz, value_type=int),
            'yolo_conf': ParameterValue(yolo_conf, value_type=float),
            'detection_rate_slam': ParameterValue(
                detection_rate_slam, value_type=float),
            'detection_rate_navigation': ParameterValue(
                detection_rate_navigation, value_type=float),
            'detection_enabled': ParameterValue(
                detection_enabled, value_type=bool),
            'preview_enabled': ParameterValue(
                preview_enabled, value_type=bool),
            'publish_annotated_image': ParameterValue(
                publish_annotated_image, value_type=bool),
            'save_video': ParameterValue(save_video, value_type=bool),
            'every_n': ParameterValue(every_n, value_type=int),
            'navigation_backend': 'qbot_astar',
            'rtabmap_mode_services': False,
            'return_strategy': 'qbot_astar',
            'laser_frame': laser_frame,
            'object_clearance': ParameterValue(
                object_clearance, value_type=float),
            'robot_radius': ParameterValue(robot_radius, value_type=float),
            'object_dedup_enabled': ParameterValue(
                object_dedup_enabled, value_type=bool),
            'object_dedup_distance': ParameterValue(
                object_dedup_distance, value_type=float),
            'object_dedup_same_class_only': ParameterValue(
                object_dedup_same_class_only, value_type=bool),
            'object_dedup_update_position': ParameterValue(
                object_dedup_update_position, value_type=bool),
            'object_lidar_fusion': ParameterValue(
                object_lidar_fusion, value_type=bool),
            'object_lidar_window_deg': ParameterValue(
                object_lidar_window_deg, value_type=float),
            'object_lidar_max_delta': ParameterValue(
                object_lidar_max_delta, value_type=float),
        }],
        condition=IfCondition(use_semantic),
    )
    delayed_semantic = TimerAction(period=6.0, actions=[semantic_navigator])

    voice_commander = Node(
        package='diff_drive_robot',
        executable='voice_commander',
        name='voice_commander',
        output='screen',
        condition=IfCondition(use_voice),
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': ParameterValue(rosbridge_port, value_type=int),
        }],
        condition=IfCondition(use_rosbridge),
    )

    web_dashboard = Node(
        package='diff_drive_robot',
        executable='web_dashboard_server',
        name='web_dashboard_server',
        output='screen',
        arguments=['--host', '0.0.0.0', '--port', web_dashboard_port],
        condition=IfCondition(use_web_dashboard),
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
        wheel_diameter_arg,
        wheel_separation_arg,
        ticks_per_rev_arg,
        left_wheel_scale_arg,
        right_wheel_scale_arg,
        swap_encoders_arg,
        invert_left_encoder_arg,
        invert_right_encoder_arg,
        straight_correction_enabled_arg,
        start_lidar_driver_arg,
        lidar_driver_package_arg,
        lidar_driver_executable_arg,
        lidar_product_name_arg,
        lidar_port_arg,
        lidar_baud_arg,
        laser_scan_dir_arg,
        scan_topic_arg,
        laser_frame_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_roll_arg,
        laser_pitch_arg,
        laser_yaw_arg,
        start_kinect_driver_arg,
        kinect_device_index_arg,
        camera_backend_arg,
        camera_driver_package_arg,
        camera_driver_executable_arg,
        use_kinect_topic_bridge_arg,
        kinect_rgb_topic_arg,
        kinect_depth_topic_arg,
        kinect_info_topic_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        yolo_model_arg,
        tracker_cfg_arg,
        image_topic_arg,
        yolo_imgsz_arg,
        yolo_conf_arg,
        detection_rate_slam_arg,
        detection_rate_navigation_arg,
        detection_enabled_arg,
        preview_enabled_arg,
        publish_annotated_image_arg,
        save_video_arg,
        every_n_arg,
        use_rviz_arg,
        use_voice_arg,
        use_semantic_arg,
        use_qbot_nav_arg,
        use_rosbridge_arg,
        rosbridge_port_arg,
        use_web_dashboard_arg,
        web_dashboard_port_arg,
        qbot_linear_speed_arg,
        qbot_max_angular_arg,
        object_clearance_arg,
        robot_radius_arg,
        object_dedup_enabled_arg,
        object_dedup_distance_arg,
        object_dedup_same_class_only_arg,
        object_dedup_update_position_arg,
        object_lidar_fusion_arg,
        object_lidar_window_arg,
        object_lidar_max_delta_arg,
        kobuki_driver,
        lidar_driver,
        robot_state_publisher,
        tf_base_to_laser,
        tf_base_to_camera,
        tf_camera_to_depth_optical,
        tf_camera_to_rgb_optical,
        freenect_camera_driver,
        openni2_camera_driver,
        kinect_topic_bridge,
        depth_preview,
        slam_node,
        slam_lifecycle_manager,
        slam_pose_publisher,
        navigation_server,
        qbot_controller,
        delayed_semantic,
        voice_commander,
        rosbridge,
        web_dashboard,
        rviz,
    ])
