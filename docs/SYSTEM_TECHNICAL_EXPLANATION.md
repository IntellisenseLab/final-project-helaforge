# Technical System Explanation

This document explains the architecture of the Kobuki LiDAR + Kinect semantic navigation system. It is intended for presentations, reports, and future maintainers.

## 1. Goal

The system turns a real Kobuki base into a semantic navigation robot:

1. Build a 2D map using LiDAR.
2. Detect objects using Kinect RGB images and YOLO.
3. Estimate object positions using Kinect depth and LiDAR range correction.
4. Pin objects in the SLAM map.
5. Navigate home or to saved objects.
6. Accept voice commands from a laptop over ROS 2 DDS.

## 2. Main Runtime Placement

```mermaid
flowchart LR
    subgraph Laptop["Laptop"]
        VC["voice_commander<br/>Vosk speech recognition"]
        T["optional arrow_teleop"]
    end

    subgraph DDS["ROS 2 DDS Network"]
        C["/semantic_nav/command"]
        CMD["/cmd_vel"]
    end

    subgraph Pi["Raspberry Pi Robot Computer"]
        SN["semantic_navigator"]
        KD["kobuki_driver"]
        LD["ldlidar_stl_ros2"]
        KF["kinect_v1_freenect_driver"]
        KB["kinect_topic_bridge"]
        SLAM["slam_toolbox"]
        SP["slam_pose_publisher"]
        A["qbot_navigation_server<br/>A* planner"]
        Q["qbot_controller<br/>path follower"]
    end

    subgraph Hardware["Robot Hardware"]
        KOB["Kobuki base"]
        LIDAR["LD19 LiDAR"]
        KIN["Kinect v1"]
    end

    VC --> C
    T --> CMD
    C --> SN
    SN --> A
    A --> Q
    Q --> CMD
    CMD --> KD
    KD --> KOB
    KOB --> KD
    LIDAR --> LD
    KIN --> KF
    KF --> KB
    LD --> SLAM
    KD --> SLAM
    SLAM --> SP
    SP --> A
    KB --> SN
```

The laptop does not need direct access to robot hardware. It only publishes recognized text commands on `/semantic_nav/command`.

## 3. Topic-Level Architecture

```mermaid
flowchart TD
    LIDAR["LD19 LiDAR"] -->|serial USB| LDL["ldlidar_stl_ros2"]
    LDL -->|sensor_msgs/LaserScan| SCAN["/scan"]

    KOB["Kobuki base"] -->|encoder packets| KDRV["kobuki_driver"]
    KDRV -->|nav_msgs/Odometry| ODOM["/odom"]
    KDRV -->|TF| TF1["odom -> base_footprint"]
    CMD["/cmd_vel"] --> KDRV

    SCAN --> ST["slam_toolbox"]
    ODOM --> ST
    TF1 --> ST
    ST --> MAP["/map"]
    ST --> TF2["map -> odom"]

    TF2 --> SPP["slam_pose_publisher"]
    TF1 --> SPP
    SPP --> SP["/slam_pose"]

    MAP --> ASTAR["qbot_navigation_server"]
    SP --> ASTAR
    GOAL["/ui_goal"] --> ASTAR
    ASTAR --> PATH["/planned_path"]
    PATH --> CTRL["qbot_controller"]
    SP --> CTRL
    CTRL --> CMD

    KIN["Kinect v1"] -->|RGB/depth USB| KDRV2["kinect_v1_freenect_driver"]
    KDRV2 --> RGB0["/camera/rgb/image_raw"]
    KDRV2 --> DEP0["/camera/depth_registered/image_raw"]
    KDRV2 --> INFO0["/camera/rgb/camera_info"]
    RGB0 --> BR["kinect_topic_bridge"]
    DEP0 --> BR
    INFO0 --> BR
    BR --> RGB["/camera/image_raw"]
    BR --> DEP["/camera/depth/image_raw"]
    BR --> INFO["/camera/camera_info"]

    RGB --> SEM["semantic_navigator"]
    DEP --> SEM
    INFO --> SEM
    SCAN --> SEM
    SP --> SEM
    CMDIN["/semantic_nav/command"] --> SEM
    SEM --> MARKERS["/semantic_nav/object_markers"]
    SEM --> STATUS["/semantic_nav/status"]
    SEM --> GOAL
```

## 4. Command State Machine

```mermaid
stateDiagram-v2
    [*] --> Idle

    Idle --> Scanning: start mapping / scan environment / start
    Scanning --> ReturningHome: stop mapping / scan stop / done
    ReturningHome --> AtHome: A* reaches home
    AtHome --> NavigatingToObject: go to object
    NavigatingToObject --> AtObject: A* reaches object goal
    AtObject --> NavigatingToObject: go to another object
    AtObject --> ReturningHome: return home
    AtHome --> ReturningHome: return home
    ReturningHome --> Idle: reset or restart stack

    Scanning: teleop enabled
    Scanning: SLAM map continues building
    Scanning: YOLO object registration active

    ReturningHome: teleop disabled
    ReturningHome: home /ui_goal sent

    NavigatingToObject: object label resolved
    NavigatingToObject: /ui_goal sent near object
```

## 5. Mapping And Return-Home Sequence

```mermaid
sequenceDiagram
    participant User
    participant Voice as Laptop voice_commander
    participant Sem as semantic_navigator on Pi
    participant Teleop as arrow_teleop
    participant SLAM as slam_toolbox
    participant Nav as A* navigation
    participant Base as Kobuki

    User->>Voice: "start mapping"
    Voice->>Sem: /semantic_nav/command = scan
    Sem->>Sem: record home pose from /slam_pose
    Sem->>Teleop: /semantic_nav/teleop_enabled = true
    User->>Teleop: arrow keys
    Teleop->>Base: /cmd_vel
    Base->>SLAM: /odom + TF
    SLAM->>Sem: /map and map pose
    Sem->>Sem: detect/register objects

    User->>Voice: "stop mapping"
    Voice->>Sem: /semantic_nav/command = scan stop
    Sem->>Teleop: /semantic_nav/teleop_enabled = false
    Sem->>Nav: /ui_goal = home
    Nav->>Base: /cmd_vel along planned path
    Base-->>Sem: pose reaches home
    Sem-->>User: status = arrived home
```

## 6. Object Mapping Pipeline

```mermaid
flowchart LR
    RGB["Kinect RGB image"] --> YOLO["YOLO + BoT-SORT"]
    YOLO --> BOX["object bounding box + track id"]
    DEPTH["Kinect registered depth"] --> PIX["depth at bbox center"]
    INFO["camera intrinsics"] --> P3D["pixel to camera 3D point"]
    BOX --> PIX
    PIX --> P3D
    P3D --> TF["TF: camera frame -> laser_link/map"]
    SCAN["LiDAR /scan"] --> FUSE["range refinement on same bearing"]
    TF --> FUSE
    FUSE --> MAPPT["map-frame object coordinate"]
    MAPPT --> STORE["object_dict[label]"]
    STORE --> MARK["RViz marker /semantic_nav/object_markers"]
```

Object labels are stored as:

```text
class_trackid
```

Examples:

```text
chair_13
bottle_4
person_2
```

The voice command does not need to say the underscore. These are equivalent:

```text
go to chair thirteen
go to chair one three
go to chair 13
```

## 7. Object Navigation Clearance

The object coordinate is not used as the robot base goal directly. If the robot base drove to the object point, the body would collide with the object.

The system computes:

```text
base_goal_distance_from_object = robot_radius + object_clearance
```

Defaults:

```text
robot_radius = 0.20 m
object_clearance = 0.05 m
base goal = 0.25 m away from object
```

This means the front of the Kobuki stops about 5 cm from the object.

```mermaid
flowchart LR
    R["current robot pose"] --> D["direction to object"]
    O["object coordinate"] --> D
    D --> G["goal point<br/>robot_radius + 0.05 m before object"]
    G --> NAV["A* goal on /ui_goal"]
```

## 8. DDS Voice Distribution

ROS 2 uses DDS discovery and pub/sub communication. The laptop voice node and Pi robot nodes are separate processes on different machines but share the same ROS graph when:

- `ROS_DOMAIN_ID` is the same
- discovery is allowed across the network
- both machines can reach each other by UDP/multicast or static peer configuration

```mermaid
flowchart LR
    subgraph Laptop
        MIC["microphone"]
        VOSK["Vosk recognizer"]
        VC["voice_commander"]
    end

    subgraph Network["Wi-Fi / Ethernet<br/>DDS"]
        TOPIC["/semantic_nav/command<br/>std_msgs/String"]
    end

    subgraph RaspberryPi
        SEM["semantic_navigator"]
        ROBOT["robot stack"]
    end

    MIC --> VOSK
    VOSK --> VC
    VC --> TOPIC
    TOPIC --> SEM
    SEM --> ROBOT
```

Recommended basic environment on both machines:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
```

If multicast is blocked, use static peers:

```bash
# Laptop
export ROS_STATIC_PEERS='PI_IP_ADDRESS'

# Pi
export ROS_STATIC_PEERS='LAPTOP_IP_ADDRESS'
```

## 9. Important Nodes

| Node | Machine | Purpose |
|---|---|---|
| `voice_commander` | Laptop | Converts speech to normalized text commands |
| `kobuki_driver` | Pi | Serial bridge between ROS `/cmd_vel` and Kobuki base |
| `ldlidar_stl_ros2_node` | Pi | Publishes `/scan` from LD19 LiDAR |
| `kinect_v1_freenect_driver` | Pi | Publishes Kinect RGB/depth streams |
| `kinect_topic_bridge` | Pi | Normalizes Kinect topics to `/camera/*` |
| `slam_toolbox` | Pi | Builds `/map` from `/scan` and `/odom` |
| `slam_pose_publisher` | Pi | Publishes `/slam_pose` from TF |
| `semantic_navigator` | Pi | Command state machine, object mapping, object goals |
| `qbot_navigation_server` | Pi | A* path planning on `/map` |
| `qbot_controller` | Pi | Path following, publishes `/cmd_vel` |

## 10. Failure Modes And Fixes

| Symptom | Likely Cause | Fix |
|---|---|---|
| No `/map` | `/scan` or `/odom` missing | Check LiDAR and Kobuki topics first |
| No `/scan` | LiDAR port wrong or permission issue | Check `/dev/serial/by-id`, udev, `dialout` |
| No `/odom` | Kobuki port wrong or base off | Check by-id device and Kobuki power |
| Kinect invalid index | Kinect busy or USB reset needed | Kill old Kinect process, unplug/replug Kinect |
| Object marker offset | Camera TF/depth issue | Verify Kinect frame, depth stream, and object is visible to LiDAR |
| Voice not seen on Pi | DDS discovery issue | Match `ROS_DOMAIN_ID`, unset `ROS_LOCALHOST_ONLY`, use static peers |
| `chair_13` not recognized | Spoken underscore/numbers | Say `chair thirteen` or `chair one three` |

## 11. Launch Summary

Pi:

```bash
ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  use_voice:=false \
  start_kinect_driver:=true \
  use_kinect_topic_bridge:=true \
  use_semantic:=true \
  use_qbot_nav:=true \
  use_rviz:=false
```

Laptop:

```bash
ros2 run diff_drive_robot voice_commander
```

Manual command test from either machine:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'start mapping'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'stop mapping'}"
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'go to chair thirteen'}"
```
