Viewed README.md:1-216

Based on the project's documentation (`README.md`), you ran the ROS 2 simulation using a multi-terminal setup. You used 4 to 5 separate terminals to launch the different components of the robot. 

For each terminal, you always start by sourcing your workspace:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
```

Here is exactly how you ran each part of the project:

### Terminal 1: Launch Gazebo Simulator
You started the 3D world with the furniture/objects and spawned the robot:
```bash
ros2 launch diff_drive_robot spawn_robot.launch.py world:=yolo_world.sdf
```

### Terminal 2: Start Navigation & SLAM
After waiting about 5 seconds for Gazebo to load, you launched the SLAM toolbox and Nav2 stack to handle mapping and localization:
```bash
ros2 launch diff_drive_robot navigation.launch.py use_sim_time:=True use_slam:=True
```

### Terminal 3: Run the Semantic Brain Node
This is the main Python node you have been working on (`semantic_navigator.py`). It handles the YOLO object detection, PID movement, and logic:
```bash
ros2 run diff_drive_robot semantic_navigator
```

### Terminal 4: Start Manual Teleop Control
You used this custom script to drive the robot around with your arrow keys during the "scanning" phase:
```bash
ros2 run diff_drive_robot arrow_teleop
```

### Terminal 5: Voice Commands (Optional)
If you were testing the offline voice control (Vosk), you ran this node to speak commands like "go to chair 1" instead of publishing them manually:
```bash
ros2 run diff_drive_robot voice_commander
```

---

### How you commanded it:
If you weren't using voice commands, you told the robot what to do by publishing string messages to its command topic from any terminal:
- **Start scanning:** `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'scan'"`
- **Stop scanning:** `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'scan stop'"`
- **List objects:** `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'list'"`
- **Go to object:** `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'chair_5'"`
