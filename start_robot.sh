#!/bin/bash
# =============================================================================
# start_robot.sh
# =============================================================================
# One-command robot launcher that auto-detects any Kobuki, LD19, and Kinect
# unit plugged in. Works even when different hardware units are swapped between
# evaluation sessions.
#
# Usage:
#   bash start_robot.sh              # detect hardware + launch full stack
#   bash start_robot.sh --detect     # detect only, do not launch
#   bash start_robot.sh --no-kinect  # skip Kinect (launch without camera)
#
# Environment overrides (optional):
#   KOBUKI_PORT=/dev/ttyUSB0  bash start_robot.sh
#   LIDAR_PORT=/dev/ttyUSB1   bash start_robot.sh
#   KINECT_INDEX=0            bash start_robot.sh
# =============================================================================
set -e

WS="/home/pi/robot_ws"
DETECT_SCRIPT="$WS/detect_hardware.py"

# ── Colour output ─────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✅  $*${NC}"; }
warn() { echo -e "${YELLOW}  ⚠️   $*${NC}"; }
err()  { echo -e "${RED}  ❌  $*${NC}"; }

# ── Parse arguments ───────────────────────────────────────────────────────────
DETECT_ONLY=false
NO_KINECT=false
for arg in "$@"; do
  case "$arg" in
    --detect)    DETECT_ONLY=true ;;
    --no-kinect) NO_KINECT=true ;;
    --help|-h)
      echo "Usage: bash start_robot.sh [--detect] [--no-kinect]"
      echo "  --detect      Show detected hardware and exit (do not launch)"
      echo "  --no-kinect   Launch without Kinect camera"
      exit 0 ;;
  esac
done

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║         diff_drive_robot — Smart Launcher            ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── Source ROS ────────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# ── Auto-detect hardware ──────────────────────────────────────────────────────
echo "Detecting hardware..."
eval "$(python3 "$DETECT_SCRIPT" --export --quiet 2>/dev/null)" || true

# Allow environment overrides (for manual testing)
KOBUKI_PORT="${KOBUKI_PORT:-$ROBOT_KOBUKI_PORT}"
LIDAR_PORT="${LIDAR_PORT:-$ROBOT_LIDAR_PORT}"
KINECT_INDEX="${KINECT_INDEX:-$ROBOT_KINECT_INDEX}"
KINECT_COUNT="${KINECT_COUNT:-$ROBOT_KINECT_COUNT}"

# ── Report what was found ─────────────────────────────────────────────────────
echo ""
echo "  ── Detected Hardware ──────────────────────────────"
if [ -n "$KOBUKI_PORT" ]; then
  ok "Kobuki   → $KOBUKI_PORT"
else
  err "Kobuki   → NOT FOUND"
fi

if [ -n "$LIDAR_PORT" ]; then
  ok "LD19     → $LIDAR_PORT"
else
  err "LD19     → NOT FOUND"
fi

if [ "${KINECT_COUNT:-0}" -gt 0 ] 2>/dev/null; then
  ok "Kinect   → device index $KINECT_INDEX ($KINECT_COUNT found)"
elif "$NO_KINECT"; then
  warn "Kinect   → skipped (--no-kinect)"
else
  err "Kinect   → NOT FOUND"
fi
echo "  ────────────────────────────────────────────────────"
echo ""

# ── Exit if detect-only mode ──────────────────────────────────────────────────
if "$DETECT_ONLY"; then
  echo "Detect-only mode — exiting without launch."
  exit 0
fi

# ── Validate required hardware ────────────────────────────────────────────────
ABORT=false
if [ -z "$KOBUKI_PORT" ]; then
  err "Kobuki not found. Plug in the Kobuki USB cable and retry."
  ABORT=true
fi
if [ -z "$LIDAR_PORT" ]; then
  err "LD19 LiDAR not found. Plug in the LiDAR USB cable and retry."
  ABORT=true
fi
if [ "${KINECT_COUNT:-0}" -eq 0 ] && ! "$NO_KINECT"; then
  err "Kinect not found. Plug in the Kinect (+ power adapter) and retry."
  err "Or use:  bash start_robot.sh --no-kinect"
  ABORT=true
fi

if "$ABORT"; then
  echo ""
  err "Cannot launch — fix the above and re-run this script."
  exit 1
fi

# ── Build launch arguments ────────────────────────────────────────────────────
KINECT_ARG_START="true"
KINECT_IDX_ARG="${KINECT_INDEX:-0}"
if "$NO_KINECT"; then
  KINECT_ARG_START="false"
fi

# Verify NCNN model is in the workspace
YOLO_MODEL="yolo26n_ncnn_model"
if [ ! -d "$WS/$YOLO_MODEL" ]; then
  warn "NCNN model not found at $WS/$YOLO_MODEL"
  warn "Trying to locate it..."
  FOUND_MODEL=$(find /home/pi -name "yolo26n_ncnn_model" -type d 2>/dev/null | head -1)
  if [ -n "$FOUND_MODEL" ]; then
    YOLO_MODEL="$FOUND_MODEL"
    ok "Found model at: $YOLO_MODEL"
  else
    err "YOLO NCNN model not found. Run: cd $WS && yolo export model=yolo26n.pt format=ncnn imgsz=640"
    exit 1
  fi
fi

# ── Launch ────────────────────────────────────────────────────────────────────
echo "Launching robot stack..."
echo ""
echo "  Kobuki port   : $KOBUKI_PORT"
echo "  LiDAR port    : $LIDAR_PORT"
echo "  Kinect index  : $KINECT_IDX_ARG"
echo "  YOLO model    : $YOLO_MODEL"
echo "  RMW           : $RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo ""
echo "  Web dashboard : http://$(hostname -I | awk '{print $1}'):8080"
echo "  Rosbridge     : ws://$(hostname -I | awk '{print $1}'):9090"
echo ""
echo "Press Ctrl+C to stop the robot."
echo "════════════════════════════════════════════════════════"
echo ""

cd "$WS"
exec ros2 launch diff_drive_robot lidar_semantic_hw.launch.py \
  serial_port:="$KOBUKI_PORT" \
  lidar_port:="$LIDAR_PORT" \
  start_kinect_driver:="$KINECT_ARG_START" \
  kinect_device_index:="$KINECT_IDX_ARG" \
  camera_backend:=freenect \
  use_voice:=false \
  use_rosbridge:=true \
  use_web_dashboard:=true \
  use_rviz:=false \
  yolo_model:="$YOLO_MODEL" \
  yolo_imgsz:=640 \
  yolo_conf:=0.40 \
  detection_rate_slam:=3.0 \
  detection_rate_navigation:=5.0
