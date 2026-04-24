# 🤖 RUNBOT — Semantic Navigation System

> ✅ **Last verified working:** 2026-04-24 — Ubuntu 24.04 / ROS 2 Jazzy

---

## 📋 Table of Contents
1. [One-Time Setup](#one-time-setup)
2. [Pre-Flight Checks](#pre-flight-checks) ← **Run these before every session**
3. [Run Simulation (Gazebo)](#run-simulation-gazebo)
4. [Run Physical Robot](#run-physical-robot-kobuki--kinect)
5. [Workflow — Commands](#workflow--commands)
6. [Troubleshooting](#troubleshooting)

---

## 🔧 One-Time Setup

> Only needed the first time or after a fresh clone.

### Build the workspace
```bash
cd ~/Music/final-project-helaforge
source /opt/ros/jazzy/setup.bash
colcon build --packages-select diff_drive_robot --symlink-install
source install/setup.bash
```

### Install Python dependencies
```bash
pip3 install pyserial ultralytics vosk sounddevice --break-system-packages
```

### Install Kinect driver (freenect)
```bash
# Build libfreenect
cd ~
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect && mkdir build && cd build
cmake -L .. -DBUILD_PYTHON3=ON
make -j4
sudo make install
sudo ldconfig /usr/local/lib64/

# Fetch audio firmware
cd ~/libfreenect
python3 ./src/fwfetcher.py
sudo mkdir -p /usr/local/share/libfreenect
sudo mv ./audios.bin /usr/local/share/libfreenect/

# Install Python wrapper
cd ~/libfreenect/wrappers/python
sudo python3 setup.py install
```

> ⚠️ **If you have a Kinect for Windows (`02c2` motor):** The K4W patch has already been applied to `~/libfreenect/src/usb_libusb10.c`. If you reclone libfreenect, see the Troubleshooting section.

### Make Kobuki serial port permanent
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in (or reboot)
```

### Create map output folder
```bash
mkdir -p ~/Documents/ROS_final_project
```

---

## ✅ Pre-Flight Checks

**Run these every session before launching.**

### 1. Source the workspace
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash
```

### 2. Check Kobuki is connected
```bash
ls /dev/ttyUSB*
# Expected: /dev/ttyUSB0

sudo chmod 666 /dev/ttyUSB0
```

### 3. Test Kobuki driver
```bash
# Run driver (keep open in a terminal)
ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0 2>&1
```
✅ Expected:
```
[kobuki_driver]: Kobuki connected on /dev/ttyUSB0
[kobuki_driver]: KobukiDriver ready.
```

In a second terminal, verify odometry is streaming:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /odom
# Expected: average rate: ~10 Hz
```

Send a test velocity command (wheels should spin):
```bash
ros2 topic pub --times 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```
> ⚠️ Lift the robot or place it with clear space before sending velocity commands.

### 4. Check Kinect is connected (all 3 USB devices)
```bash
lsusb | grep "045e"
# Must show all 3:
#   045e:02ae  Xbox NUI Camera
#   045e:02ad  Xbox NUI Audio
#   045e:02c2  Kinect for Windows NUI Motor
```

### 5. Test Kinect camera feed
```bash
python3 -c "
import freenect
frame, ts = freenect.sync_get_video()
print('OK — RGB frame shape:', frame.shape)
freenect.sync_stop()
"
```
✅ Expected: `OK — RGB frame shape: (480, 640, 3)`

---

## 🖥️ Run Simulation (Gazebo)

### Terminal 1 — Launch Gazebo + Robot
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 launch diff_drive_robot spawn_robot.launch.py
```
✅ Gazebo opens with the robot in the room world.

To use a different world:
```bash
ros2 launch diff_drive_robot spawn_robot.launch.py world:=room.sdf
```

### Terminal 2 — Semantic Navigator
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot semantic_navigator
```
✅ Expected:
```
[semantic_navigator]: YOLO loaded ✓
=== Semantic Navigator CLI ===
Commands: scan | stop scan | list | <object_id>
```

### Terminal 3 — Keyboard Teleop (to drive in simulation)
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot arrow_teleop
```
> Use `↑ ↓ ← →` arrow keys. Press `Q` to quit.

---

## 🤖 Run Physical Robot (Kobuki + Kinect)

> Complete Pre-Flight Checks above before starting.

Open **4 terminals** and run **in this order**.

### Terminal 1 — Kobuki Driver
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0 2>&1
```
✅ `[kobuki_driver]: KobukiDriver ready.`

### Terminal 2 — Kinect Bridge
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot kinect_bridge 2>&1
```
✅ `[kinect_bridge]: KinectBridge started — publishing RGB + Depth at 15 Hz`

### Terminal 3 — Semantic Navigator
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot semantic_navigator 2>&1
```
✅ `=== Semantic Navigator CLI ===` — **type commands directly here**

### Terminal 4 — Input (choose one)

**Option A — Voice commands:**
```bash
ros2 run diff_drive_robot voice_commander 2>&1
```

**Option B — Keyboard driving:**
```bash
ros2 run diff_drive_robot arrow_teleop 2>&1
```
> Use `↑ ↓ ← →` arrow keys. Press `Q` to quit.

---

## 🎮 Workflow — Commands

Once all terminals are running:

| Step | Action | Where |
|---|---|---|
| 1 | Type `scan` | Terminal 3 |
| 2 | Drive around room | Terminal 4 (arrow keys) |
| 3 | Watch object detections appear | Terminal 3 |
| 4 | Type `list` to see all found objects | Terminal 3 |
| 5 | Type `chair_1` (or any object ID) to navigate to it | Terminal 3 |
| 6 | Type `stop scan` to stop and return home | Terminal 3 |

**Voice commands (Terminal 4 voice mode):** say `"scan"`, `"list"`, `"go to chair 1"`, `"stop scan"`

---

## 🛠️ Troubleshooting

| Problem | Fix |
|---|---|
| `Cannot open /dev/ttyUSB0` | `sudo chmod 666 /dev/ttyUSB0` |
| `kobuki_driver` exits immediately, `Fatal: Parameter(s) already declared: ['use_sim_time']` | Remove `self.declare_parameter('use_sim_time', False)` from `kobuki_driver.py` — already fixed in this repo |
| Kinect: only 1 or 2 USB devices show in `lsusb` | Unplug ALL Kinect cables, wait 10s, replug. Try a different USB port. |
| `LIBUSB_ERROR_IO` / `Failed to set the LED of K4W` | K4W audio patch missing. See **K4W Patch** note below. |
| Kinect Python test fails | Run `sudo python3 ~/libfreenect/wrappers/python/setup.py install` |
| `/odom` not publishing | kobuki_driver not connected — check Terminal 1 for errors |
| Robot doesn't move | Check `ros2 topic hz /cmd_vel` — must be >0 Hz when driving |
| YOLO not detecting | Ensure good lighting; keep objects 0.5–5.0 m from Kinect |
| `vosk model not found` | `wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip *.zip && mv vosk-model-small-en-us-0.15 ~/vosk-model` |

### K4W Patch (if recloning libfreenect)

The Kinect for Windows motor (`045e:02c2`) triggers a bug where libfreenect forces audio open, causing `LIBUSB_ERROR_IO` on the camera. The fix is already applied in `~/libfreenect/src/usb_libusb10.c`. If you reclone, manually comment out lines ~447-450 in `usb_libusb10.c`:

```c
// Comment out this block:
// if ((requested_devices & FREENECT_DEVICE_MOTOR) && (requested_devices & FREENECT_DEVICE_AUDIO) == 0)
// {
//     ctx->enabled_subdevices = (freenect_device_flags)(ctx->enabled_subdevices | FREENECT_DEVICE_AUDIO);
// }
```
Then rebuild: `cd ~/libfreenect/build && make -j4 && sudo make install && sudo ldconfig /usr/local/lib64/`
