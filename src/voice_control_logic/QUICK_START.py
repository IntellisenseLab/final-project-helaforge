setup#!/usr/bin/env python3
"""
QUICK START: Python-Based Kobuki Voice Control System
======================================================

This replaces the complex C++ ROS 2 Jazzy build with a lightweight Python approach.

SETUP INSTRUCTIONS
==================

1. Prerequisites Already Installed:
   ✓ pyserial (for USB communication)
   ✓ ROS 2 Jazzy with rclpy
   ✓ User added to 'dialout' group

2. When You Plug In the Kobuki Robot:
   Run these ONE TIME to set USB permissions:
   
   $ sudo chmod 666 /dev/ttyUSB0
   
   (Or if that doesn't work, replug the robot after running setup)

3. Build the voice_control_logic package:
   
   $ cd ~/qbot_ws
   $ colcon build --packages-select voice_control_logic
   $ source install/setup.bash

RUNNING THE SYSTEM
==================

OPTION 1: Using the Launch File (RECOMMENDED)
   
   $ cd ~/qbot_ws
   $ source install/setup.bash
   $ ros2 launch voice_control_logic voice_control.launch.py
   
   This starts all three nodes:
   - Voice recognition (listening to microphone)
   - Command interpreter (speech → Kobuki commands)
   - Kobuki driver (commands → USB serial → robot)

OPTION 2: Running Individual Nodes
   
   Terminal 1 - Voice Recognition:
   $ ros2 run voice_control_logic voice_to_text
   
   Terminal 2 - Command Interpreter:
   $ ros2 run voice_control_logic voice_command_interpreter
   
   Terminal 3 - Kobuki Driver:
   $ ros2 run voice_control_logic simple_kobuki_driver

VOICE COMMANDS AVAILABLE
=========================

"forward"            → Move robot forward
"backward" / "back"  → Move robot backward
"left" / "turn left" → Rotate left
"right"/ "turn right"→ Rotate right
"spin"               → Spin in place
"stop" / "halt"      → Stop immediately

TROUBLESHOOTING
===============

❌ Problem: "No such file or directory: '/dev/ttyUSB0'"
   Solution: Plug in the Kobuki and grant permissions:
   $ sudo chmod 666 /dev/ttyUSB0

❌ Problem: Permission denied accessing /dev/ttyUSB0
   Solution: Add yourself to dialout group:
   $ sudo usermod -a -G dialout $USER
   $ # Then log out and log back in (or restart)

❌ Problem: Voice recognition not detecting speech
   Solution: Check microphone:
   $ arecord -t wav -c 1 -r 16000 -d 5 /tmp/test.wav
   $ aplay /tmp/test.wav

SYSTEM ARCHITECTURE
===================

Voice Recognition (voice_to_text.py)
         ↓ publishes /recognized_speech (String)
         ↓
Command Interpreter (voice_command_interpreter.py)
         ↓ publishes /cmd_vel (Twist)
         ↓
Kobuki Driver (simple_kobuki_driver.py)
         ↓ sends serial commands over /dev/ttyUSB0
         ↓
Kobuki Robot 🤖

WHY THIS APPROACH?
==================

Original Problem:
- ROS 2 Jazzy doesn't have binary packages for kobuki_node
- Building from source requires 45+ dependencies (ECL/Sophus math libs)
- These have complex version conflicts and weren't available for Ubuntu 24.04

Our Solution (Following CS3340 Project Guidelines):
- Lightweight Python serial driver (~200 lines of code)
- Direct USB communication (no complex ROS 2 middleware)
- Easy to debug and modify
- Follows "pragmatic robotics" approach mentioned in your PDF

REFERENCE: CS3340 Project Guidelines Section 1.2
"Flexible robot communication methods appropriate to the task"

DEVELOPMENT NOTES
=================

The Kobuki protocol uses:
- Serial port: /dev/ttyUSB0 at 115,200 baud
- Header format: [0xAA, 0x55, Length, Command, Data..., Checksum]
- Commands available in simple_kobuki_driver.py

To add new commands:
1. Edit voice_command_interpreter.py to recognize new voice phrases
2. Edit simple_kobuki_driver.py to add new motor commands
3. Rebuild: colcon build --packages-select voice_control_logic

TESTING WITHOUT ROBOT
=====================

To test the system without a physical robot connected:

Terminal 1:
$ ros2 run voice_control_logic voice_command_interpreter

Terminal 2 (send test command):
$ ros2 topic pub --once /recognized_speech std_msgs/msg/String "data: forward"

Terminal 3 (monitor velocity commands):
$ ros2 topic echo /cmd_vel

You should see your voice command converted to /cmd_vel messages!
"""

# Setup script - run this once on first time use
import os
import subprocess
import sys


def setup_usb_permissions():
    """Configure USB permissions for Kobuki communication"""
    print("\n🔧 Setting up USB permissions for Kobuki...")
    
    # Add user to dialout group
    result = subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', os.environ['USER']], 
                          capture_output=True)
    if result.returncode == 0:
        print("✓ Added user to 'dialout' group")
    else:
        print("✗ Failed to add user to 'dialout' group")
        print("  Try manually: sudo usermod -a -G dialout $USER")
    
    print("\n✓ Setup complete!")
    print("\nNext steps:")
    print("1. Plug in the Kobuki robot")
    print("2. Run: cd ~/qbot_ws && colcon build")
    print("3. Run: source install/setup.bash")
    print("4. Run: ros2 launch voice_control_logic voice_control.launch.py")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'setup':
        setup_usb_permissions()
    else:
        print(__doc__)
