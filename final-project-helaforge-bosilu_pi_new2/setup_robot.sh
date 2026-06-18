#!/bin/bash
# =============================================================================
# setup_robot.sh
# One-shot setup script for the diff_drive_robot ROS 2 stack
# Run with: bash setup_robot.sh
# =============================================================================
set -e

REPO_SRC="/home/pi/Desktop/web-based final project/final-project-helaforge-bosilu_pi_new2/src"
WS="/home/pi/robot_ws"
BASHRC="$HOME/.bashrc"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║       diff_drive_robot — Robot Stack Setup           ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── Step 1: Source ROS 2 ──────────────────────────────────────────────────────
echo "[1/7] Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# ── Step 2: Create workspace symlinks ────────────────────────────────────────
echo "[2/7] Setting up ~/robot_ws workspace..."
mkdir -p "$WS/src"
for pkg in diff_drive_robot interfaces ldlidar_stl_ros2 oasis_kinect2; do
  TARGET="$WS/src/$pkg"
  if [ ! -e "$TARGET" ]; then
    ln -s "$REPO_SRC/$pkg" "$TARGET"
    echo "  linked: $pkg"
  else
    echo "  already exists: $pkg"
  fi
done

# ── Step 3: Install apt dependencies ─────────────────────────────────────────
echo "[3/7] Installing missing ROS 2 packages..."
sudo apt-get install -y \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-image-view \
  ros-jazzy-openni2-camera \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-slam \
  ros-jazzy-rtabmap-sync \
  ros-jazzy-rtabmap-viz \
  libfreenect-dev \
  freenect 2>&1 | grep -E '(Setting up|already|ERROR|error)' || true

# ── Step 4: udev rules ───────────────────────────────────────────────────────
echo "[4/7] Writing udev rules..."

sudo tee /etc/udev/rules.d/99-kobuki.rules > /dev/null << 'EOF'
# Kobuki robot base — stable symlink /dev/kobuki
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="kobuki*", SYMLINK+="kobuki", MODE="0666"
EOF

sudo tee /etc/udev/rules.d/99-ld19.rules > /dev/null << 'EOF'
# LD19 LiDAR (CP2102 / CH340) — stable symlink /dev/ld19
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ld19", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ld19", MODE="0666"
EOF

sudo tee /etc/udev/rules.d/66-kinect.rules > /dev/null << 'EOF'
# Microsoft Kinect v1 / Xbox 360 Kinect — libfreenect USB access
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
EOF

sudo usermod -aG video,plugdev "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  udev rules written and reloaded"

# ── Step 5: Build the workspace ───────────────────────────────────────────────
echo "[5/7] Building robot_ws with colcon..."
cd "$WS"
colcon build --symlink-install 2>&1 | tee /tmp/robot_build.log
BUILD_EXIT=${PIPESTATUS[0]}

if [ $BUILD_EXIT -ne 0 ]; then
  echo ""
  echo "❌ BUILD FAILED — last 30 lines of log:"
  tail -30 /tmp/robot_build.log
  exit 1
fi

echo "  ✅ Build complete"

# ── Step 6: Update ~/.bashrc ──────────────────────────────────────────────────
echo "[6/7] Updating ~/.bashrc..."

if ! grep -q "robot_ws/install/setup.bash" "$BASHRC"; then
  echo "" >> "$BASHRC"
  echo "# robot_ws — diff_drive_robot stack" >> "$BASHRC"
  echo "source /home/pi/robot_ws/install/setup.bash" >> "$BASHRC"
  echo "export ROS_DOMAIN_ID=42" >> "$BASHRC"
  echo "  added robot_ws source and ROS_DOMAIN_ID=42 to .bashrc"
else
  echo "  already in .bashrc"
fi

# ── Step 7: Final status ──────────────────────────────────────────────────────
echo ""
echo "[7/7] Checking what is ready..."
source "$WS/install/setup.bash"
echo ""
echo "Installed packages in robot_ws:"
ros2 pkg list | grep diff_drive_robot && echo "  ✅ diff_drive_robot" || echo "  ❌ diff_drive_robot missing"
ros2 pkg list | grep interfaces && echo "  ✅ interfaces" || echo "  ❌ interfaces missing"
ros2 pkg list | grep ldlidar_stl && echo "  ✅ ldlidar_stl_ros2" || echo "  ❌ ldlidar_stl_ros2 missing"

echo ""
echo "════════════════════════════════════════════════════════"
echo " ✅ Setup complete! Now:"
echo ""
echo " 1. Log out and back in (for plugdev group to take effect)"
echo " 2. Plug in Kobuki + LiDAR + Kinect"
echo " 3. Check: ls /dev/kobuki /dev/ld19"
echo " 4. Export YOLO model (if not done):"
echo "      cd ~/robot_ws && yolo export model=yolo26n.pt format=ncnn imgsz=640"
echo " 5. Launch the robot:"
echo "      cd ~/robot_ws"
echo "      source install/setup.bash"
echo "      ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \\"
echo "        use_voice:=false use_rosbridge:=true use_web_dashboard:=true \\"
echo "        use_rviz:=false yolo_model:=yolo26n_ncnn_model \\"
echo "        yolo_imgsz:=640 yolo_conf:=0.40 \\"
echo "        detection_rate_slam:=3.0 detection_rate_navigation:=5.0"
echo "════════════════════════════════════════════════════════"
