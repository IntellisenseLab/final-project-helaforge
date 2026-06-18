# Python-Based Kobuki Voice Control System

This is a lightweight, Python-based alternative to the complex C++ ROS 2 Jazzy Kobuki build. It replaces ~45 dependencies with ~200 lines of straightforward Python code.

## Why This Solution?

**Problem:** Standard `kobuki_node` ROS 2 Jazzy binaries don't exist for Ubuntu 24.04, and building from source requires:
- Complex ECL/Sophus math library dependencies
- Version conflicts on modern Ubuntu
- 45+ packages to compile

**Solution:** Direct USB serial communication using lightweight Python drivers, as recommended in **CS3340 Project Guidelines Section 1.2**.

---

## Quick Start

### 1️⃣ Initial Setup (Run Once)

```bash
cd ~/qbot_ws
colcon build --packages-select voice_control_logic
source install/setup.bash
```

### 2️⃣ USB Permissions (When Kobuki is Plugged In)

```bash
sudo chmod 666 /dev/ttyUSB0
```

### 3️⃣ Start the System

```bash
# All-in-one launch (recommended):
ros2 launch voice_control_logic voice_control.launch.py

# OR run individually in separate terminals:
# Terminal 1:
ros2 run voice_control_logic voice_to_text

# Terminal 2:
ros2 run voice_control_logic voice_command_interpreter

# Terminal 3:
ros2 run voice_control_logic simple_kobuki_driver
```

---

## Available Voice Commands

| Command | Action |
|---------|--------|
| "forward", "go", "move forward" | Move forward |
| "backward", "back", "reverse" | Move backward |
| "left", "turn left" | Turn left |
| "right", "turn right" | Turn right |
| "spin", "rotate", "turn around" | Spin 360° |
| "stop", "halt" | Stop immediately |

---

## System Architecture

```
┌─────────────────────────────────┐
│   Voice-to-Text Node            │  Listens to microphone
│ (Vosk speech recognition)       │  Publishes: /recognized_speech
└────────────┬────────────────────┘
             │
             ↓ (String: voice command)
┌─────────────────────────────────┐
│   Command Interpreter Node      │  Converts speech → commands
│ (voice_command_interpreter.py)  │  Publishes: /cmd_vel (Twist)
└────────────┬────────────────────┘
             │
             ↓ (Twist: linear & angular velocity)
┌─────────────────────────────────┐
│   Simple Kobuki Driver          │  Converts ROS → Serial protocol
│ (simple_kobuki_driver.py)       │  Sends: Kobuki protocol bytes
└────────────┬────────────────────┘
             │
             ↓ (Serial: /dev/ttyUSB0 @ 115,200 baud)
┌─────────────────────────────────┐
│   Kobuki Robot 🤖                │  Receives motor commands
│ (via USB-to-Serial)             │  Drives wheels
└─────────────────────────────────┘
```

---

## File Structure

```
voice_control_logic/
├── voice_control_logic/
│   ├── __init__.py
│   ├── voice_to_text.py                  # Speech recognition
│   ├── voice_command_interpreter.py      # Speech → velocity commands
│   ├── simple_kobuki_driver.py           # ROS → USB serial protocol
│   └── test_mic.py
├── launch/
│   └── voice_control.launch.py           # Start all 3 nodes
├── setup.py                              # Package configuration
├── package.xml
└── README.md                             # This file
```

---

## Testing Without a Robot

You can test the full pipeline without physical hardware:

**Terminal 1 - Monitor velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Terminal 2 - Command interpreter:**
```bash
ros2 run voice_control_logic voice_command_interpreter
```

**Terminal 3 - Send a test command:**
```bash
ros2 topic pub --once /recognized_speech std_msgs/msg/String "data: forward"
```

Expected output in Terminal 1:
```
linear:
  x: 0.30000000000000004
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `/dev/ttyUSB0` not found | Plug in Kobuki, run: `sudo chmod 666 /dev/ttyUSB0` |
| Permission denied on `/dev/ttyUSB0` | `sudo usermod -a -G dialout $USER` then restart |
| Microphone not picking up speech | Test with: `arecord -t wav -c 1 -r 16000 -d 5 /tmp/test.wav && aplay /tmp/test.wav` |
| Command not recognized | Check the interpreter accepts your command in `voice_command_interpreter.py` |

---

## Implementation Details

### Kobuki Serial Protocol

The driver implements the Kobuki command protocol:

```
[HEADER1] [HEADER2] [LENGTH] [COMMAND] [DATA...] [CHECKSUM]
 0xAA      0x55       byte     byte    N bytes   XOR all
```

Base control command (0x01):
- Data: 2×2 bytes for left/right wheel velocity (mm/s)
- Range: -500 to +500 mm/s per wheel

### Differential Drive Kinematics

The system converts linear (m/s) + angular (rad/s) velocities to individual wheel commands:

```python
v_left  = v_linear - (ω_angular × L/2)
v_right = v_linear + (ω_angular × L/2)
```

where `L = 230mm` (wheel separation for Kobuki).

---

## For Your CS3340 Project Report

**Challenge Identified:** ROS 2 Jazzy dependency chain for Kobuki (45+ packages, ECL/Sophus conflicts)

**Resolution:** Implemented lightweight Python-based driver following project guidelines' recommendation for "flexible robot communication methods appropriate to the task"

**Benefits:**
- ✓ Runs immediately (no 30+ minute compilation)
- ✓ Easy to debug and modify
- ✓ ~200 lines of transparent code
- ✓ Direct hardware control
- ✓ Demonstrates embedded systems understanding

**Keywords:** Serial protocol, differential drive kinematics, ROS 2 topic-based architecture, pragmatic robotics

---

## References

- Kobuki Serial Protocol: See `simple_kobuki_driver.py` docstrings
- ROS 2 Documentation: https://docs.ros.org/en/jazzy/
- Vosk Speech Recognition: https://alphacephei.com/vosk/

---

## License

Same as CS3340 project specification
