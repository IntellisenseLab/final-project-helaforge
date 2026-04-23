# 🤖 Running the Physical Robot (Kobuki + Kinect)

This guide explains how to run the semantic navigation system on the **physical Kobuki robot** using a **Kinect camera** — no simulation required.

---

## 📦 Prerequisites

### 1. ROS 2 & Workspace
Make sure ROS 2 Jazzy and the workspace are built:
```bash
cd ~/Music/final-project-helaforge
source /opt/ros/jazzy/setup.bash
colcon build --packages-select diff_drive_robot --symlink-install
source install/setup.bash
```

### 2. Python Dependencies
```bash
pip3 install pyserial ultralytics vosk sounddevice --break-system-packages
```

### 3. Kinect Python Wrapper (freenect)
> Follow the full setup guide in the project Wiki.
> Once done, verify with:
```bash
python3 -c 'import freenect; print("Kinect wrapper OK")'
```

### 4. Create the map output folder
```bash
mkdir -p ~/Documents/ROS_final_project
```

---

## 🔌 Hardware Setup

### Plug in devices
| Device | Port |
|---|---|
| Kobuki robot | USB → usually `/dev/ttyUSB0` |
| Kinect camera | USB (3 separate USB connections: Motor, Camera, Audio) |

### Verify hardware is detected
```bash
# Kobuki serial port
ls /dev/ttyUSB*

# Kinect USB devices
lsusb | grep "045e"
# Should show 3 devices: 02ae (Camera), 02b0 (Motor), 02ad (Audio)

# Grant serial access without sudo
sudo chmod 666 /dev/ttyUSB0
```

### Test Kinect is working
```bash
# Visual test (opens a window)
freenect-glview

# Camera test only
freenect-camtest
```

---

## 🚀 Launch Sequence

Open **4 terminals** and run in this exact order.

---

### Terminal 1 — Kobuki Driver
Connects to the physical robot over serial. Subscribes to `/cmd_vel` to drive wheels and publishes `/odom` from wheel encoders.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0
```

✅ Expected output:
```
[kobuki_driver]: Kobuki connected on /dev/ttyUSB0
[kobuki_driver]: KobukiDriver ready.
```

---

### Terminal 2 — Kinect Bridge
Reads RGB and Depth frames from the physical Kinect and publishes them as ROS 2 image topics.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot kinect_bridge
```

✅ Expected output:
```
[kinect_bridge]: KinectBridge started — publishing RGB + Depth at 15 Hz
```

> ⚠️ If you see `Error: Can't open device` — make sure the Kinect is plugged in and you have run:
> ```bash
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```

---

### Terminal 3 — Semantic Navigator (Brain)
Runs YOLO object detection, records object positions, and drives the robot autonomously. **Type commands directly here.**

```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot semantic_navigator
```

✅ Expected output:
```
[semantic_navigator]: YOLO loaded ✓
[semantic_navigator]: Ready. Commands: scan | stop scan | list | <object_id>

=== Semantic Navigator CLI ===
Commands: scan | stop scan | list | <object_id>
```

---

### Terminal 4 — Input Method (choose one)

**Option A: Voice Commands (microphone)**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot voice_commander
```

**Option B: Manual Keyboard Driving**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Music/final-project-helaforge/install/setup.bash

ros2 run diff_drive_robot arrow_teleop
```
> Use `↑ ↓ ← →` arrow keys to drive. Press `Q` to quit.

---

## 🎮 Full Workflow

### Step 1 — Start Scanning
In **Terminal 3**, type:
```
scan
```
Or say **"scan"** into the microphone (Terminal 4 voice mode).

The robot will start logging any objects it sees through the Kinect.

---

### Step 2 — Drive Around the Room
Use **Terminal 4** (arrow teleop) to manually drive the robot around the environment.

Point the Kinect at objects like chairs, bottles, people, etc.

You will see detections appear in Terminal 3:
```
[semantic_navigator]: NEW OBJECT: chair_1   dist=1.2m  pos=(2.31, 0.88)
[semantic_navigator]: NEW OBJECT: bottle_1  dist=0.8m  pos=(1.10, -0.45)
```

---

### Step 3 — List Detected Objects
In **Terminal 3**, type:
```
list
```
Output:
```
  1. chair_1      → (+2.31, +0.88)
  2. bottle_1     → (+1.10, -0.45)
```

---

### Step 4 — Navigate to an Object
In **Terminal 3**, type the object ID:
```
chair_1
```
Or say **"go to chair 1"** (voice mode).

The robot will autonomously drive to 1 metre in front of `chair_1` and stop.

```
[semantic_navigator]: Navigating to chair_1 …
[semantic_navigator]: Arrived at chair_1
```

---

### Step 5 — Return Home
In **Terminal 3**, type:
```
stop scan
```
Or say **"stop scan"**.

The robot will retrace its path back to where it started.

---

## ✅ Topic Verification

You can verify everything is connected correctly in a new terminal:

```bash
source /opt/ros/jazzy/setup.bash

# Check all topics are publishing
ros2 topic list

# Verify odometry rate (~10 Hz)
ros2 topic hz /odom

# Verify Kinect camera rate (~15 Hz)
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw

# Monitor velocity commands when navigating
ros2 topic echo /cmd_vel

# Monitor detected object commands
ros2 topic echo /semantic_nav/command
```

---

## 🛠️ Troubleshooting

| Problem | Fix |
|---|---|
| `Cannot open /dev/ttyUSB0` | Run `sudo chmod 666 /dev/ttyUSB0` |
| `Error: Can't open device` (Kinect) | Unplug and replug Kinect; run `sudo udevadm trigger` |
| `freenect not found` | Run `sudo python3 ~/libfreenect/wrappers/python/setup.py install` |
| Robot doesn't move | Check `/cmd_vel` is publishing; verify kobuki_driver is connected |
| YOLO not detecting objects | Ensure good lighting; objects must be 0.5–5.0 m from Kinect |
| `vosk model not found` | Download: `wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip && unzip *.zip && mv vosk-model-small-en-us-0.15 ~/vosk-model` |

---

## 📁 Map Persistence

Detected objects are automatically saved to:
```
~/Documents/ROS_final_project/yolo_semantic_map.json
```

This map is reloaded automatically the next time you run `semantic_navigator`, so previously detected objects are remembered across sessions.

---

## 🏗️ System Architecture

```
Physical Kinect
      │ (freenect Python wrapper)
      ▼
kinect_bridge.py ──► /camera/image_raw
                 ──► /camera/depth/image_raw
                 ──► /camera/camera_info

Physical Kobuki
      │ (serial 115200 baud)
      ▼
kobuki_driver.py ──► /odom  (wheel encoder odometry)
                 ◄── /cmd_vel (velocity commands)

voice_commander.py ──► /semantic_nav/command
  OR
stdin CLI          ──► /semantic_nav/command

                         ▼
               semantic_navigator.py
               ├── YOLO detection on /camera/image_raw
               ├── depth estimation from /camera/depth/image_raw
               ├── position tracking from /odom
               ├── obstacle avoidance from /scan (optional LiDAR)
               ├── object map: ~/Documents/ROS_final_project/yolo_semantic_map.json
               └── PID navigation ──► /cmd_vel ──► Kobuki wheels
```

---

## 🍓 Running on Raspberry Pi 5 (4 GB)

This section covers everything you need to set up and run the system on a **Raspberry Pi 5 (4 GB)** — on top of an existing ROS 2 Jazzy + Gazebo installation.

> **Note:** The RPi 5 will run **headless** (no monitor/RViz). You control it via SSH from your laptop.

---

### Step 1 — Clone the workspace on the Pi

```bash
# On the Raspberry Pi (via SSH):
cd ~
git clone https://github.com/SudilMin/final-project-helaforge.git
cd final-project-helaforge
```

---

### Step 2 — Install additional Python packages

```bash
pip3 install pyserial ultralytics vosk sounddevice --break-system-packages
```

> ⚠️ **YOLO on RPi 5 (4 GB):** `ultralytics` will use CPU inference which is slow.
> To speed it up, increase `YOLO_EVERY_N` in `semantic_navigator.py` from `5` to `15`
> so YOLO only runs every 15 frames instead of every 5.

---

### Step 3 — Install Kinect drivers (libfreenect)

```bash
# Install build dependencies
sudo apt-get install -y git-core cmake freeglut3-dev pkg-config build-essential \
  libxmu-dev libxi-dev libusb-1.0-0-dev cython3 python-dev-is-python3 python3-numpy

# Clone and build libfreenect
cd ~
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect && mkdir build && cd build
cmake -L ..
make
sudo make install
sudo ldconfig /usr/local/lib64/

# Add udev rules for non-root access
sudo nano /etc/udev/rules.d/51-kinect.rules
```

Paste into the file:
```
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```

```bash
# Fetch audio firmware (fixes LIBUSB_ERROR_IO)
cd ~/libfreenect
python3 ./src/fwfetcher.py
sudo mkdir -p /usr/local/share/libfreenect
sudo mv ./audios.bin /usr/local/share/libfreenect/
cd build && sudo make install

# Add user to groups
sudo adduser $USER video
sudo adduser $USER plugdev

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

### Step 4 — Install the freenect Python wrapper

```bash
# Replace setup.py with the fixed setuptools version
cd ~/libfreenect/wrappers/python
sudo nano setup.py   # paste the fixed setup.py from the project Wiki
sudo python3 setup.py install

# Verify
python3 -c 'import freenect; print("Kinect wrapper OK")'
```

---

### Step 5 — Download the Vosk speech model

```bash
cd ~
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 ~/vosk-model
rm vosk-model-small-en-us-0.15.zip
```

---

### Step 6 — Build the ROS 2 workspace

```bash
cd ~/final-project-helaforge
source /opt/ros/jazzy/setup.bash
colcon build --packages-select diff_drive_robot --symlink-install
source install/setup.bash
```

---

### Step 7 — Set up permissions for Kobuki serial port

```bash
# Find the port
ls /dev/ttyUSB*

# Grant access
sudo chmod 666 /dev/ttyUSB0

# Make it permanent (survives reboot)
sudo usermod -a -G dialout $USER
# Then log out and log back in (or reboot)
```

---

### Step 8 — Create output folder

```bash
mkdir -p ~/Documents/ROS_final_project
```

---

### Step 9 — Run the system (headless via SSH)

Open **4 SSH terminals** from your laptop into the Pi.

> **Connect via SSH:**
> ```bash
> ssh pi@<raspberry-pi-ip>
> # e.g. ssh pi@192.168.1.100
> ```

**SSH Terminal 1 — Kobuki Driver:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/final-project-helaforge/install/setup.bash
ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0
```

**SSH Terminal 2 — Kinect Bridge:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/final-project-helaforge/install/setup.bash
ros2 run diff_drive_robot kinect_bridge
```

**SSH Terminal 3 — Semantic Navigator:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/final-project-helaforge/install/setup.bash
ros2 run diff_drive_robot semantic_navigator
```
Type commands directly: `scan`, `list`, `chair_1`, `stop scan`

**SSH Terminal 4 — Voice Commander:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/final-project-helaforge/install/setup.bash
ros2 run diff_drive_robot voice_commander
```

---

### RPi 5 Performance Tips

| Tip | Command / Setting |
|---|---|
| Run YOLO less often (faster) | Change `YOLO_EVERY_N = 15` in `semantic_navigator.py` |
| Check CPU temperature | `vcgencmd measure_temp` |
| Check RAM usage | `free -h` |
| Use `tmux` to keep sessions alive over SSH | `sudo apt install tmux` then `tmux new -s robot` |
| Auto-start on boot | Use `systemd` service or add to `/etc/rc.local` |

---

### RPi 5 Troubleshooting

| Problem | Fix |
|---|---|
| `pip3 install` fails | Add `--break-system-packages` flag |
| YOLO too slow | Increase `YOLO_EVERY_N` to `20` in `semantic_navigator.py` |
| Out of memory | Close other processes; reboot the Pi |
| SSH connection drops | Use `tmux` to keep sessions alive |
| `Cannot open /dev/ttyUSB0` | Run `sudo chmod 666 /dev/ttyUSB0` |
| Kinect not detected | Replug USB; run `sudo udevadm trigger` |
| `freenect not found` | Re-run `sudo python3 ~/libfreenect/wrappers/python/setup.py install` |

---

