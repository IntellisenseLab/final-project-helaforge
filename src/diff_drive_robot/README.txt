# QBot2 Semantic Robot Navigator

A ROS 2 Jazzy autonomous robot navigation system that integrates **YOLOv8 object detection**, **SORT multi-object tracking**, **SLAM mapping**, **PID motion control with obstacle avoidance**, and **Vosk voice commands** to enable a QBot2 differential-drive robot to scan an environment, identify objects, and autonomously navigate to them.

## Features

- **Semantic Scanning** – Drive the robot around manually; YOLO detects and registers objects with their map coordinates
- **Autonomous Return-Home** – Odometry-based waypoint retracing using PID controller
- **Object Navigation** – Say or type an object name; the robot navigates to it and stops 50 cm away
- **Obstacle Avoidance** – LiDAR-based reactive safety layer (hard stop, slowdown, side steering)
- **Voice Commands** – Offline speech recognition using Vosk (no internet needed)
- **Arrow-Key Teleop** – Custom teleop with stop-on-release behavior

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim8) |
| Python | 3.12 |
| CUDA (optional) | For GPU-accelerated YOLO |

---

## Installation

### 1. Install ROS 2 Jazzy

Follow [ROS 2 Jazzy Install Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### 2. Install ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-cv-bridge \
  ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-rviz2
```

### 3. Install Python Dependencies

```bash
pip3 install ultralytics filterpy lapx opencv-python vosk sounddevice --break-system-packages
```

> **Note:** If you hit a NumPy conflict, run: `pip3 install "numpy<2" --break-system-packages`

### 4. Download Vosk Speech Model

```bash
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 vosk-model
```

> Or set a custom path: `export VOSK_MODEL_PATH=/path/to/your/model`

### 5. Clone and Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url> diff_drive_robot
cd ~/ros2_ws
colcon build --packages-select diff_drive_robot
source install/setup.bash
```

---

## How to Run

You need **4–5 terminals**. In each one, first run:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
```

### Terminal 1 – Gazebo Simulator

```bash
ros2 launch diff_drive_robot spawn_robot.launch.py world:=yolo_world.sdf
```

### Terminal 2 – Navigation + SLAM (wait ~5 seconds after T1)

```bash
ros2 launch diff_drive_robot navigation.launch.py use_sim_time:=True use_slam:=True
```

### Terminal 3 – SemanticNavigator (brain node)

```bash
ros2 run diff_drive_robot semantic_navigator
```

### Terminal 4 – Arrow-Key Teleop (for manual driving during scan)

```bash
ros2 run diff_drive_robot arrow_teleop
```

### Terminal 5 – Voice Commander (optional, replaces manual topic pub)

```bash
ros2 run diff_drive_robot voice_commander
```

---

## Commands

Commands can be issued via **voice** (Terminal 5) or **topic publish** (any terminal):

| Command | Voice Phrase | Topic Publish |
|---|---|---|
| Start scanning | "scan" | `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'scan'"` |
| Stop scanning | "stop scan" | `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'scan stop'"` |
| List objects | "list" | `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'list'"` |
| Go to object | "go to chair_5" | `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'chair_5'"` |
| Return home | "return home" | `ros2 topic pub --once /semantic_nav/command std_msgs/String "data: 'return home'"` |

---

## Workflow

1. **Start scanning** – say "scan" or publish the scan command
2. **Drive around** – use arrow keys in Terminal 4 to explore the environment
3. **Objects appear** – YOLO detects objects and logs them with coordinates
4. **Stop scanning** – say "stop scan"; robot retraces waypoints back to start
5. **View object list** – say "list" to see all detected objects
6. **Navigate to object** – say "go to chair_5"; robot drives there and stops 50 cm away
7. **Chain navigation** – you can go to multiple objects one after another
8. **Return home** – say "return home" at any time to go back to starting position

---

## Project Structure

```
diff_drive_robot/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── diff_drive_robot              # ament index marker
├── diff_drive_robot/                  # Python package (nodes + library)
│   ├── __init__.py
│   ├── semantic_navigator.py          # Main brain node (YOLO+SORT+PID+Nav)
│   ├── voice_commander.py             # Vosk voice command interface
│   ├── arrow_teleop.py                # Arrow-key manual control
│   ├── yolo_tracker.py                # Standalone YOLO+SORT node
│   ├── sort.py                        # SORT tracking algorithm
│   └── odom_to_tf.py                  # Odometry → TF broadcaster
├── config/
│   ├── gz_bridge.yaml                 # Gazebo ↔ ROS 2 topic bridge
│   ├── mapper_params_online_async.yaml # SLAM Toolbox config
│   ├── nav2_params.yaml               # Nav2 parameters
│   └── nav2_rviz.rviz                 # RViz config
├── launch/
│   ├── spawn_robot.launch.py          # Gazebo + robot URDF
│   ├── navigation.launch.py           # SLAM + Nav2 stack
│   └── slam.launch.py                 # SLAM-only launch
├── urdf/
│   └── robot.urdf.xacro               # QBot2 robot model + sensors
└── worlds/
    └── yolo_world.sdf                 # Gazebo world with furniture
```

---

## Key Design Decisions

- **No depth camera required** – Object positions estimated from bbox size + robot odometry
- **Odometry-based return-home** – Records breadcrumb waypoints every 60 cm, retraces them using PID; more reliable than Nav2 in unknown environments
- **LiDAR obstacle avoidance** – Three-zone system: hard stop (<30 cm), proportional slowdown (<70 cm), side steering (<80 cm)
- **PID controller** – Full PID (P+I+D) for both linear and angular velocity with velocity ramping for smooth acceleration/deceleration
- **Throttled YOLO** – Runs every 10th frame to reduce GPU load; no OpenCV display window
- **Vosk offline** – No internet required for voice commands

---

## Troubleshooting

| Issue | Fix |
|---|---|
| `Module 'ultralytics' not found` | `pip3 install ultralytics --break-system-packages` |
| `numpy` version conflict | `pip3 install "numpy<2" --break-system-packages` |
| Voice commander: no microphone | Check `arecord -l` and set default device |
| Vosk model not found | Download and extract to `~/vosk-model` |
| Robot spins but doesn't move | Restart Gazebo – physics may have frozen |
| SLAM `malformed map` errors | These are non-critical during initial map building |

---

## License

MIT
