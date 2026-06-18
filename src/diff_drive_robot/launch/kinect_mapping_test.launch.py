"""
kinect_mapping_test.launch.py
=============================
Kinect-only test launch for RGB preview, depth preview, and RTAB-Map 2D mapping.

This launch does not start Kobuki, Nav2, voice control, or semantic navigation.
It is for checking that the connected Kinect v1 can publish RGB-D data and that
RTAB-Map can build a 2D occupancy grid from Kinect depth using RGB-D odometry.
Keep the Kinect approximately level for the 2D grid to look like a floor map.
RGB and depth are synchronized through rtabmap_sync/rgbd_sync before odometry.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('diff_drive_robot')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'kinect_2d_map.rviz')

    use_image_view_arg = DeclareLaunchArgument(
        'use_image_view', default_value='true',
        description='Open live RGB and depth preview windows')

    use_rtabmap_viz_arg = DeclareLaunchArgument(
        'use_rtabmap_viz', default_value='true',
        description='Open RTAB-Map visualization')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz with the Kinect 2D map visualization config')

    camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='0.24',
        description='Kinect height above the virtual base_footprint frame')

    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='0.0',
        description='Kinect pitch in radians; positive tilts sensor downward')

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/kinect_mapping_test.db'),
        description='RTAB-Map database path for this Kinect-only test')

    use_image_view = LaunchConfiguration('use_image_view')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    use_rviz = LaunchConfiguration('use_rviz')
    database_path = LaunchConfiguration('database_path')
    camera_height = LaunchConfiguration('camera_height')
    camera_pitch = LaunchConfiguration('camera_pitch')

    kinect_driver = Node(
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
    )

    topic_bridge = Node(
        package='diff_drive_robot',
        executable='kinect_topic_bridge',
        name='kinect_topic_bridge',
        output='screen',
        parameters=[{
            'source_rgb_topic': '/camera/rgb/image_raw',
            'source_depth_topic': '/camera/depth_registered/image_raw',
            'source_camera_info_topic': '/camera/rgb/camera_info',
            'target_rgb_topic': '/camera/image_raw',
            'target_depth_topic': '/camera/depth/image_raw',
            'target_camera_info_topic': '/camera/camera_info',
            'output_frame_id': 'camera_rgb_optical_frame',
        }],
    )

    tf_base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint_to_base_link',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', camera_height,
            '--roll', '0.0', '--pitch', camera_pitch, '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
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

    rgb_preview = Node(
        package='image_view',
        executable='image_view',
        name='kinect_rgb_preview',
        output='screen',
        remappings=[('image', '/camera/image_raw')],
        condition=IfCondition(use_image_view),
    )

    depth_preview_converter = Node(
        package='diff_drive_robot',
        executable='depth_preview',
        name='depth_preview',
        output='screen',
        parameters=[{
            'input_topic': '/camera/depth/image_raw',
            'output_topic': '/camera/depth/preview',
            'min_depth': 0.5,
            'max_depth': 4.0,
            'invert': True,
        }],
        condition=IfCondition(use_image_view),
    )

    depth_preview = Node(
        package='image_view',
        executable='image_view',
        name='kinect_depth_preview',
        output='screen',
        remappings=[('image', '/camera/depth/preview')],
        condition=IfCondition(use_image_view),
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
            'camera_height': ParameterValue(camera_height, value_type=float),
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
    )

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

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'publish_tf_map': False,
            'subscribe_rgbd': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_odom_info': True,
            'approx_sync': False,
            'qos_image': 1,
            'qos_camera_info': 1,
            'Vis/FeatureType': '9',
            'Vis/MaxFeatures': '500',
            'Kp/DetectorStrategy': '9',
            'Reg/Strategy': '0',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
            'Odom/AlignWithGround': 'true',
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom'),
        ],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_rgbd': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_odom_info': True,
            'subscribe_scan': False,
            'approx_sync': False,
            'database_path': database_path,
            'qos_image': 1,
            'qos_camera_info': 1,
            'Grid/FromDepth': 'true',
            'Grid/RangeMin': '0.5',
            'Grid/RangeMax': '3.5',
            'Grid/DepthDecimation': '4',
            'Grid/MaxObstacleHeight': '1.4',
            'Grid/MinGroundHeight': '-0.05',
            'Grid/MaxGroundHeight': '0.08',
            'Grid/RayTracing': 'true',
            'Grid/CellSize': '0.07',
            'Grid/NormalsSegmentation': 'false',
            'Grid/NoiseFilteringRadius': '0.10',
            'Grid/NoiseFilteringMinNeighbors': '2',
            'Grid/ClusterRadius': '0.12',
            'Grid/MinClusterSize': '10',
            'Grid/3D': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'Rtabmap/DetectionRate': '1',
            'Reg/Force3DoF': 'true',
            'Optimizer/Slam2D': 'true',
            'RGBD/LinearUpdate': '0.10',
            'RGBD/AngularUpdate': '0.10',
            'RGBD/OptimizeFromGraphEnd': 'true',
            'RGBD/ProximityBySpace': 'true',
            'Kp/DetectorStrategy': '9',
            'Reg/Strategy': '0',
            'Vis/FeatureType': '9',
            'Vis/MaxFeatures': '600',
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom'),
            ('map', '/map'),
        ],
        arguments=['--delete_db_on_start'],
    )

    delayed_mapping = TimerAction(period=3.0, actions=[rgbd_odometry, rtabmap])

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'subscribe_rgbd': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'approx_sync': False,
            'qos_image': 1,
            'qos_camera_info': 1,
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom'),
        ],
        condition=IfCondition(use_rtabmap_viz),
    )

    delayed_rtabmap_viz = TimerAction(period=5.0, actions=[rtabmap_viz])

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
        use_image_view_arg,
        use_rtabmap_viz_arg,
        use_rviz_arg,
        database_path_arg,
        camera_height_arg,
        camera_pitch_arg,
        kinect_driver,
        topic_bridge,
        tf_base_to_link,
        tf_base_to_camera,
        tf_camera_to_rgb_optical,
        tf_camera_to_depth_optical,
        rgb_preview,
        depth_preview_converter,
        depth_preview,
        kinect_depth_grid,
        rgbd_sync,
        delayed_mapping,
        delayed_rtabmap_viz,
        rviz,
    ])
