# QBot2 Semantic Navigation: Work Procedure & RTAB-Map Integration

This document outlines the workflow and architectural decisions made to successfully integrate **RTAB-Map (Real-Time Appearance-Based Mapping)** alongside **RViz2** for the QBot2 Semantic Navigation project.

## 1. Why RTAB-Map and RViz Together?

While **RViz** is excellent for visualizing the robot's state (TF tree, LiDAR scans, YOLO bounding boxes, and planned paths), it is only a visualizer—it does not build maps. 

To achieve persistent memory of the environment, we integrated **RTAB-Map**, an advanced RGB-D SLAM system.
- **RTAB-Map** runs in the background, consuming camera depth data and odometry to generate a high-density 3D point cloud of the environment.
- **RViz** is used alongside it to monitor the robot's real-time ROS 2 navigation stack (Nav2) and our custom `semantic_navigator` markers.

## 2. Resolving the Odometry Conflict (The "Red Screen" Bug)

During integration, we encountered a critical failure where RTAB-Map would turn solid red, indicating it had lost odometry tracking the moment the robot moved. 

**The Root Cause:**
By default, RTAB-Map attempts to compute its own **Visual Odometry** (`rgbd_odometry`) using the camera feed. In a simulator like Gazebo, plain walls lack the micro-textures (features) that cameras need to track movement. 
Furthermore, our `spawn_robot.launch.py` was launching `odom_to_tf`, which uses the physical wheel encoders to broadcast the robot's position (`odom -> base_footprint`). Because both the camera and the wheels were publishing the exact same TF link, a massive conflict occurred, crashing the TF tree.

**The Solution:**
We enforced a strictly **Wheel Odometry-based SLAM** architecture:
1. **Disabled Visual Odometry:** In `navigation.launch.py`, we explicitly set `'visual_odometry': 'false'`.
2. **Linked Wheel Encoders:** We set `'odom_frame_id': 'odom'`, forcing RTAB-Map to trust the `odom_to_tf` node for spatial tracking.

By doing this, the robot perfectly tracks its movement using wheel encoders (which never fail in an empty room), while RTAB-Map focuses entirely on using the camera feed to build the beautiful 3D map.

## 3. How to Run the Full Stack

To run the complete system with both RViz and RTAB-Map, we use a 5-terminal workflow. Open 5 terminals and source your workspace in each:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
```

**Terminal 1: Start the Simulation**
Spawns the Gazebo world and the QBot2 robot.
```bash
ros2 launch diff_drive_robot spawn_robot.launch.py world:=yolo_world.sdf
```

**Terminal 2: Start SLAM & Navigation (RTAB-Map + RViz)**
Launches the Nav2 stack, RTAB-Map for 3D SLAM, and RViz for visualization.
```bash
ros2 launch diff_drive_robot navigation.launch.py use_sim_time:=True use_slam:=True
```
*(You will see both the RViz window and the RTAB-Map window open simultaneously).*

**Terminal 3: Start Semantic Brain**
Launches the custom YOLO+Depth logic that binds physical objects to the map.
```bash
ros2 run diff_drive_robot semantic_navigator
```

**Terminal 4: Teleop Control**
Used to manually drive the robot around the room to build the map and scan objects.
```bash
ros2 run diff_drive_robot arrow_teleop
```

**Terminal 5: Voice Control (Optional)**
Listens for offline Vosk voice commands like "Go to chair 1".
```bash
ros2 run diff_drive_robot voice_commander
```

## 4. Semantic Scanning Workflow

1. Start the system using the steps above.
2. Publish the scan command: `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'scan'"`
3. Drive the robot around using the arrow keys (Terminal 4). 
4. As you drive:
   - **RTAB-Map** will build a dense 3D representation of the room based on depth camera data.
   - **RViz** will show real-time 2D LiDAR hits and semantic YOLO markers appearing dynamically.
5. Stop scanning: `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'scan stop'"`
6. Tell the robot to navigate to a detected object (e.g., "chair_1"). Nav2 handles the path planning, and the custom PID controller ensures a precise 1.0m standoff distance.
