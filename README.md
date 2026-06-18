# Kobuki LiDAR + Kinect Semantic Robot

ROS 2 Jazzy real-hardware workspace for a Kobuki mobile base with:

- LD19 2D LiDAR for mapping and navigation
- Kinect v1 / Xbox 360 RGB-D camera for object recognition
- SLAM Toolbox for `/map`
- Custom A* navigation copied/adapted from the reference QBot project
- YOLO + Kinect depth + LiDAR range fusion for object pins
- Voice commands running either on the robot computer or on a separate laptop over DDS

The current recommended deployment is:

- **Raspberry Pi**: runs robot hardware, LiDAR SLAM, Kinect object mapping, navigation
- **Laptop**: runs `voice_commander` only and publishes commands to the Pi over ROS 2 DDS

For a presentation/report-level explanation, see [docs/SYSTEM_TECHNICAL_EXPLANATION.md](docs/SYSTEM_TECHNICAL_EXPLANATION.md).

## 1. Hardware

Recommended:

- Raspberry Pi 5 or Raspberry Pi 4, 64-bit
- Ubuntu Server 24.04 Noble arm64
- ROS 2 Jazzy
- Good 5V power supply for the Pi
- Powered USB hub for Kinect/LiDAR if USB power is unstable
- Kobuki base USB serial connection
- LD19 LiDAR through CP2102 USB serial adapter
- Kinect v1 / Xbox 360 sensor with power adapter

This project was tested with:

- Kobuki serial device shown as `Yujin_Robot_iClebo_Kobuki`
- LD19 LiDAR serial device shown as `Silicon_Labs_CP2102`
- Kinect v1 USB IDs `045e:02c2`, `045e:02ae`, `045e:02ad`

## 2. Install ROS 2 Jazzy On The Raspberry Pi

Install Ubuntu Server 24.04 64-bit on the Pi first. Then SSH into the Pi.

Set locale:

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Enable the ROS 2 apt repository:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo apt update

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Install ROS 2 and build tools:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  git \
  build-essential \
  cmake \
  pkg-config
```

Initialize `rosdep` if it has not been initialized before:

```bash
sudo rosdep init || true
rosdep update
```

Add ROS setup to the shell:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

## 3. Install Robot Dependencies On The Pi

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-opencv \
  ros-jazzy-image-view \
  ros-jazzy-rosbridge-server \
  ros-jazzy-openni2-camera \
  ros-jazzy-depthimage-to-laserscan \
  libfreenect-dev \
  libfreenect-bin \
  libusb-1.0-0-dev \
  portaudio19-dev
```

Install Python packages:

```bash
pip3 install --break-system-packages \
  pyserial \
  numpy \
  opencv-python \
  ultralytics \
  freenect \
  vosk \
  sounddevice
```

Performance note: YOLO on a Raspberry Pi can be slow. This project defaults to the local `yolo26n.pt` model; use `every_n:=15` or `every_n:=20` if the Pi is overloaded.

Make sure `yolo26n.pt` exists in the workspace root on the Pi:

```bash
ls -lh ~/semantic_robot_ws/yolo26n.pt
```

If the model is not committed to GitHub, copy it to the Pi manually or launch with an absolute path:

```bash
ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  yolo_model:=/absolute/path/to/yolo26n.pt
```

## 4. Clone This Repository

If your GitHub repository contains this whole workspace, including the `src/` folder:

```bash
cd ~
git clone <YOUR_GITHUB_REPO_URL> semantic_robot_ws
cd ~/semantic_robot_ws
```

If your GitHub repository contains only the ROS packages, clone it into a workspace:

```bash
mkdir -p ~/semantic_robot_ws/src
cd ~/semantic_robot_ws/src
git clone <YOUR_GITHUB_REPO_URL>
cd ~/semantic_robot_ws
```

Replace `<YOUR_GITHUB_REPO_URL>` with your actual GitHub URL.

## 5. Required LiDAR Driver Modification

This repository already contains the fixed `ldlidar_stl_ros2` source. If you clone this repository exactly, you should not need to modify it again.

If you replace it with the upstream LDROBOT driver and the build fails with errors about `pthread_mutex_t`, `pthread_mutex_lock`, or `pthread_mutex_unlock`, apply this patch manually.

Open:

```bash
nano src/ldlidar_stl_ros2/ldlidar_driver/include/logger/log_module.h
```

Inside the Linux block, make sure `pthread.h` is included:

```cpp
#ifndef LINUX
#include <windows.h>
#else
#include <pthread.h>
#include <stdarg.h>
#define printf_s(fileptr,str)  (fprintf(fileptr,"%s",str))
#define __in
#endif
```

Also check:

```bash
nano src/ldlidar_stl_ros2/CMakeLists.txt
```

Make sure the executable links pthread:

```cmake
target_link_libraries(${PROJECT_NAME}_node pthread)
```

Quick command-line patch:

```bash
grep -q "#include <pthread.h>" src/ldlidar_stl_ros2/ldlidar_driver/include/logger/log_module.h || \
  sed -i '/#else/a #include <pthread.h>' src/ldlidar_stl_ros2/ldlidar_driver/include/logger/log_module.h

grep -q "target_link_libraries.*pthread" src/ldlidar_stl_ros2/CMakeLists.txt || \
  sed -i '/ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs)/a target_link_libraries(${PROJECT_NAME}_node pthread)' src/ldlidar_stl_ros2/CMakeLists.txt
```

Why this is needed: the LD19 driver uses POSIX pthread mutex functions in the logger implementation. On some Linux/arm64 builds, the header is not pulled in indirectly, so the build fails unless `pthread.h` is included explicitly.

## 6. USB Permissions And Device Rules

Add your user to `dialout` so it can open `/dev/ttyUSB*`:

```bash
sudo usermod -aG dialout $USER
```

Create optional stable device rules:

```bash
sudo tee /etc/udev/rules.d/99-kobuki.rules >/dev/null <<'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", SYMLINK+="kobuki"
EOF

sudo cp src/ldlidar_stl_ros2/scripts/ldlidar.rules /etc/udev/rules.d/99-ldlidar.rules
```

Create Kinect v1 USB rules:

```bash
sudo tee /etc/udev/rules.d/51-kinect-v1.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE:="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE:="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE:="0666"
EOF
```

Reload udev and reboot:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

After reboot:

```bash
ls -l /dev/serial/by-id/* /dev/kobuki /dev/ldlidar 2>/dev/null
lsusb | grep -i microsoft
```

The launch files auto-detect Yujin Kobuki and CP2102 LiDAR by `/dev/serial/by-id`, so the by-id names are preferred.

## 7. Build On The Pi

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select interfaces ldlidar_stl_ros2 diff_drive_robot

source install/setup.bash
```

Add the workspace setup permanently:

```bash
echo "source ~/semantic_robot_ws/install/setup.bash" >> ~/.bashrc
```

## 8. Test Hardware On The Pi

### 8.1 Check USB Devices

```bash
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
lsusb | grep -Ei "microsoft|silicon|yujin|cp210|ftdi"
```

Expected:

- Kobuki: FTDI/Yujin Robot device
- LiDAR: Silicon Labs CP2102 device
- Kinect: Microsoft Kinect camera/motor/audio endpoints

### 8.2 Test LiDAR Only

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws
source install/setup.bash

ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD19 \
  -p topic_name:=scan \
  -p frame_id:=laser_link \
  -p port_name:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  -p port_baudrate:=230400
```

In another terminal:

```bash
ros2 topic hz /scan
```

If your by-id path is different, use:

```bash
ls -l /dev/serial/by-id/*
```

### 8.3 Test Kobuki Only

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws
source install/setup.bash

ros2 run diff_drive_robot kobuki_driver
```

In another terminal:

```bash
ros2 topic echo /odom
```

Slow motion test:

```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Stop with `Ctrl+C`, then:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 8.4 Test Kinect Only

```bash
freenect-glview
```

If that works, test ROS:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws
source install/setup.bash

ros2 run diff_drive_robot kinect_v1_freenect_driver
```

In another terminal:

```bash
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/depth_registered/image_raw
```

## 9. Run The Full Robot Stack On The Pi

For the Pi, do not launch RViz locally. Run visualization on the laptop if needed.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws
source install/setup.bash

ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  use_voice:=false \
  start_kinect_driver:=true \
  use_kinect_topic_bridge:=true \
  use_semantic:=true \
  use_qbot_nav:=true \
  use_rviz:=false \
  every_n:=15 \
  qbot_linear_speed:=0.08 \
  qbot_max_angular_speed:=0.25
```

Check required topics:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic echo /semantic_nav/status
```

Start mapping manually:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'start mapping'}"
```

Stop mapping and return home:

```bash
ros2 topic pub --once /semantic_nav/command std_msgs/msg/String "{data: 'stop mapping'}"
```

## 10. Distributed Voice Setup: Laptop Publishes To Pi Over DDS

The Pi should run the robot stack with:

```bash
use_voice:=false
```

The laptop runs only:

```bash
ros2 run diff_drive_robot voice_commander
```

Both machines must be on the same network and must use the same ROS domain.

### 10.1 On The Raspberry Pi

Pick a domain ID. Example: `42`.

```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" >> ~/.bashrc
echo "unset ROS_LOCALHOST_ONLY" >> ~/.bashrc

source ~/.bashrc
```

Start the robot stack:

```bash
cd ~/semantic_robot_ws
source install/setup.bash

ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  use_voice:=false \
  start_kinect_driver:=true \
  use_kinect_topic_bridge:=true \
  use_semantic:=true \
  use_qbot_nav:=true \
  use_rviz:=false
```

### 10.2 On The Laptop

Install ROS 2 Jazzy and clone/build the same repository. The laptop only needs voice dependencies, but building the same repo keeps the command node identical.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

export VOSK_MODEL_PATH=~/vosk-model
export VOICE_COMMAND_GRAMMAR=true
ros2 run diff_drive_robot voice_commander
```

Voice commands published on the laptop should appear on the Pi as:

```bash
ros2 topic echo /semantic_nav/command std_msgs/msg/String
```

### 10.3 If DDS Discovery Does Not Work

Find IP addresses:

```bash
hostname -I
```

Assume:

- Pi IP: `192.168.1.50`
- Laptop IP: `192.168.1.20`

On the Pi:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS='192.168.1.20'
unset ROS_LOCALHOST_ONLY
```

On the laptop:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS='192.168.1.50'
unset ROS_LOCALHOST_ONLY
```

Then restart both ROS processes.

Troubleshooting checklist:

- `ROS_DOMAIN_ID` must be identical on both machines.
- Do not set `ROS_LOCALHOST_ONLY=1`.
- Disable VPNs while testing.
- Allow UDP/multicast on the Wi-Fi network.
- If university Wi-Fi blocks multicast, use a phone hotspot, router, or `ROS_STATIC_PEERS`.
- Test basic DDS first:

```bash
# Laptop
ros2 topic pub --rate 1 /semantic_nav/command std_msgs/msg/String "{data: 'start mapping'}"

# Pi
ros2 topic echo /semantic_nav/command std_msgs/msg/String
```

## 11. Optional Laptop Teleop Over DDS

If you want to drive from the laptop keyboard while the Pi controls hardware, run this on the laptop with the same DDS environment:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/semantic_robot_ws
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

ros2 run diff_drive_robot arrow_teleop
```

The Pi's `semantic_navigator` publishes `/semantic_nav/teleop_enabled`, and the laptop teleop publishes `/cmd_vel` back to the Pi.

## 12. References

- ROS 2 Jazzy Ubuntu deb install docs: https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst
- ROS 2 locale setup: https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/_Ubuntu-Set-Locale.rst
- ROS 2 apt repository setup: https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/_Apt-Repositories.rst
- ROS 2 improved dynamic discovery: https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst
