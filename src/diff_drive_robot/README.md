# Kobuki LiDAR + Kinect Semantic Navigator

ROS 2 Jazzy real-hardware project for a voice-controlled autonomous robot using:

- Kobuki differential-drive base
- 2D LiDAR for environment mapping and navigation
- Kinect v1 / Xbox 360 style RGB-D sensor through libfreenect for object recognition only
- SLAM Toolbox 2D LiDAR mapping
- Friend-project A* path planner + pure-pursuit controller
- YOLO + BoT-SORT semantic object detection
- Vosk offline voice commands
- RViz visualization with live RGB, depth preview, LiDAR map, planned path, scan, and object pins

This package is now hardware-only. The main runtime path is
`lidar_semantic_hw.launch.py`. Kinect-based RTAB-Map mapping is no longer used
by the main robot stack; Kinect stays in the system for RGB-D object detection.

## Runtime Concept

1. `voice_commander` or manual terminal commands publish normalized commands on `/semantic_nav/command`.
2. `semantic_navigator` owns the robot state machine:
   - `Scan Environment`: records the map-frame home pose, enables keyboard teleop, and starts YOLO object registration while LiDAR SLAM continues mapping.
   - `Scan Stop`: disables teleop, prints/publishes the object list, and sends the copied A* controller back to the home point.
   - `Go to <object>`: resolves an exact object label or unique class name and publishes a map-frame A* goal near that object.
   - `Return Home`: publishes the recorded scan-start pose as an A* goal.
3. `kobuki_driver` bridges `/cmd_vel` to the real Kobuki serial protocol and publishes `/odom` plus `odom -> base_footprint` TF.
4. The LiDAR driver publishes `/scan`; SLAM Toolbox consumes `/scan` + `/odom` and publishes `/map` plus `map -> odom`.
5. `slam_pose_publisher` publishes `/slam_pose` from TF for the copied A* navigation stack.
6. `qbot_navigation_server` plans through `/map`; `qbot_controller` follows the path and publishes `/cmd_vel`.
7. `kinect_v1_freenect_driver` publishes Kinect v1 RGB-D topics. `kinect_topic_bridge` normalizes them to:
   - `/camera/image_raw`
   - `/camera/depth/image_raw`
   - `/camera/camera_info`
8. YOLO detections are back-projected through Kinect depth, transformed into `map`, stored, and published as `/semantic_nav/object_markers`.

## Important Topics

| Topic | Purpose |
|---|---|
| `/cmd_vel` | Velocity command to Kobuki |
| `/odom` | Kobuki wheel odometry |
| `/scan` | Real 2D LiDAR scan |
| `/map` | SLAM Toolbox 2D occupancy grid |
| `/slam_pose` | Map-frame robot pose for A* navigation |
| `/ui_goal` | Map-frame goal used by the copied A* controller |
| `/planned_path` | A* path published for visualization/web UI |
| `/camera/image_raw` | Registered Kinect RGB image |
| `/camera/depth/image_raw` | Registered Kinect depth, `32FC1` metres |
| `/camera/camera_info` | Depth/registered camera intrinsics |
| `/semantic_nav/command` | Voice/manual command input |
| `/semantic_nav/teleop_enabled` | Scan-mode keyboard teleop gate |
| `/semantic_nav/status` | Arrival, list, and scan status text |
| `/qbot_nav/status` | Copied A* controller status text |
| `/semantic_nav/object_markers` | 3D object pins and labels in RViz |

## Main LiDAR Launch

The friend project already has the LD19 driver installed in its workspace. Until
the LiDAR driver source is copied into this workspace, source that workspace
before this one:

```bash
source /opt/ros/jazzy/setup.bash
source /media/nimsika/WindowsData/semester-04/robotic-and-automation/final-project/simulation-project/qbot_navigating_to_the_goal/kobuki_ws/install/setup.bash
cd /media/nimsika/WindowsData/semester-04/robotic-and-automation/final-project/simulation-project/ros2_ws
source install/setup.bash
```

Start LiDAR mapping/navigation with Kinect object recognition:

```bash
ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  use_voice:=false
```

The launch auto-detects the Yujin Kobuki and CP2102 LiDAR by `/dev/serial/by-id`.
If auto-detection fails, check ports:

```bash
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
```

Then pass the correct port:

```bash
ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  use_voice:=false
```

Manual commands:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'scan environment'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'scan stop'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'return home'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'go to bottle'}"
```

Object pins use Kinect RGB-D direction/depth plus LiDAR range refinement on the
same bearing. The default object goal keeps the robot front `5 cm` from the
object by using `robot_radius:=0.20` and `object_clearance:=0.05`:

```bash
ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  object_lidar_fusion:=true \
  object_clearance:=0.05 \
  robot_radius:=0.20
```

Manual map-click/web-style goal test:

```bash
ros2 topic pub --once /ui_goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"
```

## Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-openni2-camera \
  ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-rtabmap-slam \
  ros-jazzy-rtabmap-viz \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-sync \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-view \
  ros-jazzy-rviz2 \
  freenect \
  libfreenect-dev \
  libfreenect-bin
```

Python dependencies:

```bash
pip3 install ultralytics opencv-python pyserial vosk sounddevice freenect --break-system-packages
```

Vosk model:

```bash
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 vosk-model
export VOSK_MODEL_PATH=~/vosk-model
```

## Kinect v1 Driver

Your connected device appears as Kinect v1:

```text
045e:02c2 Microsoft Corp. Kinect for Windows NUI Motor
045e:02ad Microsoft Corp. Xbox NUI Audio
045e:02ae Microsoft Corp. Xbox NUI Camera
```

This is not a Kinect v2. The project now defaults to `camera_backend:=freenect`
and starts `kinect_v1_freenect_driver` for this sensor. The driver publishes:

```text
/camera/rgb/image_raw
/camera/depth_registered/image_raw
/camera/rgb/camera_info
```

Then `kinect_topic_bridge` republishes them as the stable project topics:

```text
/camera/image_raw
/camera/depth/image_raw
/camera/camera_info
```

Check the device:

```bash
lsusb | grep -i microsoft
lsusb -t
```

Kinect v1 normally appears on a `480M` USB 2.0 path. That is okay for this libfreenect setup.

If `ros2 run openni2_camera list_devices` prints `Found 0 devices`, that is
expected for many Xbox/Kinect-v1 units on Jazzy because `openni2_camera` targets
OpenNI2/PrimeSense devices. Use the default freenect backend for this project.

If `kinect_v1_freenect_driver` prints `LIBUSB_ERROR_BUSY` or `Can't open device`,
stop every old Kinect launch first, then reset the USB state:

```bash
pkill -f kinect_v1_freenect_driver
pkill -f freenect
pkill -f openni
pkill -f kinect2
sudo modprobe -r gspca_kinect
sudo modprobe -r snd_usb_audio
```

Unplug the Kinect USB cable, wait 5 seconds, plug it back in, then confirm:

```bash
lsusb | grep -i microsoft
freenect-glview
```

If `freenect-glview` cannot show the camera, ROS cannot show it either.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /media/nimsika/WindowsData/semester-04/robotic-and-automation/final-project/simulation-project/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select diff_drive_robot
source install/setup.bash
```

## Full System Launch

Terminal 1, full robot stack:

```bash
source /opt/ros/jazzy/setup.bash
cd /media/nimsika/WindowsData/semester-04/robotic-and-automation/final-project/simulation-project/ros2_ws
source install/setup.bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py serial_port:=/dev/ttyUSB0
```

The default launch uses `camera_backend:=freenect`. If you start the camera
yourself in another terminal:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  start_camera_driver:=false
```

If you later use an OpenNI2/PrimeSense camera instead, switch backend:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  camera_backend:=openni2
```

Optional: if you install `matlabbe/kinect_ros2`, you can use the C++ Kinect Xbox
360 style backend that RTAB-Map examples use:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  camera_backend:=kinect_ros2
```

If your source topics differ, pass the actual topic names:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  kinect_rgb_topic:=/camera/rgb/image_raw \
  kinect_depth_topic:=/camera/depth_registered/image_raw \
  kinect_info_topic:=/camera/rgb/camera_info
```

Terminal 2, keyboard teleop:

```bash
source /opt/ros/jazzy/setup.bash
cd /media/nimsika/WindowsData/semester-04/robotic-and-automation/final-project/simulation-project/ros2_ws
source install/setup.bash
ros2 run diff_drive_robot arrow_teleop
```

The teleop terminal stays disabled until you say or publish `Scan Environment`.

## Voice and Manual Commands

Spoken phrases:

| Goal | Say |
|---|---|
| Start scan/mapping mode | `Scan Environment` |
| Stop scan and Nav2 waypoint return | `Scan Stop` |
| List detected objects | `List Objects` |
| Navigate to object | `Go to chair` or `Go to chair_7` |
| Return home | `Return Home` |

Manual equivalents:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'scan environment'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'scan stop'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'list'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'go to chair'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'return home'}"
```

Watch status:

```bash
ros2 topic echo /semantic_nav/status
```

## Individual Hardware Tests

### 1. Kobuki Base Test

Find the serial port:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

Run the driver:

```bash
ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0
```

In another terminal, verify odometry:

```bash
ros2 topic echo /odom
```

Carefully command a slow forward motion, then stop:

```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Press `Ctrl+C`, then:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 2. Kinect v1/libfreenect Test

Confirm USB sees the Kinect v1 endpoints:

```bash
lsusb | grep -i microsoft
```

Optional low-level libfreenect test:

```bash
freenect-glview
```

Run the Kinect v1 driver:

```bash
ros2 run diff_drive_robot kinect_v1_freenect_driver
```

Run the topic bridge:

```bash
ros2 run diff_drive_robot kinect_topic_bridge
```

Verify streams:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo --once /camera/camera_info
```

View RGB:

```bash
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

View all robot visualization surfaces:

```bash
rviz2 -d install/diff_drive_robot/share/diff_drive_robot/config/nav2_rviz.rviz
```

### 3. Kinect Preview + 2D RTAB-Map Test

This starts only the Kinect, live RGB preview, live depth preview, RGB-D odometry,
RTAB-Map, RViz, and RTAB-Map visualization. The launch publishes two 2D grids:

- `/kinect_depth_map`: live Kinect depth projected to a local 2D grid
- `/map`: RTAB-Map's SLAM occupancy grid, available after `/odom` works

```bash
source /opt/ros/jazzy/setup.bash
cd /media/nimsika/WindowsData/semester-04/robotic-and-automation/final-project/simulation-project/ros2_ws
source install/setup.bash
ros2 launch diff_drive_robot kinect_mapping_test.launch.py
```

Keep the Kinect level and about `0.24 m` above the floor. If your camera height
is different:

```bash
ros2 launch diff_drive_robot kinect_mapping_test.launch.py camera_height:=0.24
```

Move the Kinect/robot slowly and keep textured objects in view. Watch these topics:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/depth/preview
ros2 topic hz /rgbd_image
ros2 topic hz /kinect_depth_map
ros2 topic echo --once /odom
ros2 topic echo --once /map
```

Save the 2D occupancy grid:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/kinect_2d_map
```

### 4. Voice Control Test

Terminal 1:

```bash
ros2 topic echo /semantic_nav/command
```

Terminal 2:

```bash
export VOSK_MODEL_PATH=~/vosk-model
ros2 run diff_drive_robot voice_commander
```

Say:

```text
Scan Environment
Scan Stop
Return Home
Go to chair
```

The echo terminal should print normalized commands such as `scan`, `scan stop`, `return home`, and `chair`.

## Project Structure

```text
diff_drive_robot/
├── launch/
│   └── rtabmap_hw.launch.py
│   └── kinect_mapping_test.launch.py
├── config/
│   ├── nav2_params_hw.yaml
│   ├── rtabmap_params.yaml
│   └── nav2_rviz.rviz
└── diff_drive_robot/
    ├── kobuki_driver.py
    ├── kinect_v1_freenect_driver.py
    ├── kinect_topic_bridge.py
    ├── semantic_navigator.py
    ├── voice_commander.py
    ├── arrow_teleop.py
    └── yolo_tracker.py
```

## Tuning Notes

- Upstream patterns integrated into this project:
  - RTAB-Map ROS 2 launch style: RGB and registered depth are synchronized with `rtabmap_sync/rgbd_sync`, then RTAB-Map subscribes to `/rgbd_image`.
  - RTAB-Map Kinect Xbox 360 example style: optional C++ `kinect_ros2_node` backend is available through `camera_backend:=kinect_ros2`.
  - Kobuki upstream style: keep `odom -> base_footprint` as the wheel odometry TF and expose wheel calibration values instead of hiding them in code.
- Current physical robot: Yujin Kobuki `KAEJE11112`, Kinect mounted at the robot center, `24 cm` from ground, pitch `0`.
- Camera mount is passed to `rtabmap_hw.launch.py` with `camera_x`, `camera_y`, `camera_z`, `camera_roll`, `camera_pitch`, and `camera_yaw`. The project default is now `camera_x:=0.0 camera_y:=0.0 camera_z:=0.24 camera_pitch:=0.0`. `camera_pitch` is in radians, and positive means the Kinect is tilted downward.
- Start with a level camera. If the Kinect points down by about 10 degrees, use `camera_pitch:=0.17`. If it points down by about 15 degrees, use `camera_pitch:=0.26`.
- The local `/kinect_depth_map` is a live depth projection, not accumulated SLAM. The saved navigation map is `/map` from RTAB-Map.
- In RViz, temporarily disable `GlobalCostmap` and `LocalCostmap` when checking map quality. Look at raw `Map` (`/map`) and `KinectLiveDepthGrid` (`/kinect_depth_map`) separately.
- Nav2 robot radius is `0.20 m` in `nav2_params_hw.yaml`; update it if your Kobuki payload is wider.
- Wheel odometry is tunable at launch with `wheel_diameter`, `wheel_separation`, and `ticks_per_rev`. Bad wheel calibration makes the robot pose drift or curve around the RTAB-Map map.
- If RViz curves while you command straight forward, first check `/kobuki/encoder_debug`. If `dl_m` and `dr_m` are very different while the robot is physically moving straight, tune `left_wheel_scale` / `right_wheel_scale` or check encoder direction.
- To check straight odometry, drive exactly 1 m and compare `/odom`. If `/odom` reports too little distance, increase `wheel_diameter`; if it reports too much distance, decrease `wheel_diameter`.
- To check rotation odometry, rotate exactly 360 degrees. If `/odom` yaw reports too little rotation, decrease `wheel_separation`; if it reports too much rotation, increase `wheel_separation`.
- The default YOLO model is `yolov8n.pt`. Use a custom model with:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  yolo_model:=/absolute/path/to/model.pt
```

Example launch with common mapping tuning values:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  use_voice:=false \
  camera_x:=0.0 \
  camera_y:=0.0 \
  camera_z:=0.24 \
  camera_pitch:=0.0 \
  scan_height:=40 \
  wheel_diameter:=0.070 \
  wheel_separation:=0.230
```

Return-home behavior:

```text
return_strategy:=nav2_waypoints
```

This sends a `NavigateThroughPoses` route through the recorded map waypoints.
If it fails or times out, the robot falls back to smooth odom-PID retracing.
Use `return_strategy:=pid` only when testing without a stable Nav2 map.

PID fallback defaults:

```text
return_max_linear:=0.10
return_max_angular:=0.35
```

Use even slower values for first real-world test:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  use_voice:=false \
  return_strategy:=nav2_waypoints \
  return_max_linear:=0.07 \
  return_max_angular:=0.25
```

When you publish `scan stop`, the robot disables teleop, stops mapping, then
uses Nav2 `NavigateThroughPoses` to return through the recorded map waypoints.
If Nav2 rejects, fails, or times out, it retraces the odom waypoints using
low-speed PID with linear and angular velocity ramping.

Straight-line odometry debug:

```bash
ros2 topic echo /kobuki/encoder_debug
```

In another terminal, command slow straight motion:

```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

If debug shows `dl_m` larger than `dr_m`, reduce `left_wheel_scale` or increase
`right_wheel_scale`. If `dr_m` is larger, reduce `right_wheel_scale` or increase
`left_wheel_scale`. Example:

```bash
ros2 launch diff_drive_robot rtabmap_hw.launch.py \
  serial_port:=/dev/ttyUSB0 \
  use_voice:=false \
  left_wheel_scale:=0.98 \
  right_wheel_scale:=1.00
```

If one side shows negative metres while driving forward, try the matching invert
parameter:

```bash
invert_left_encoder:=true
```

or

```bash
invert_right_encoder:=true
```
