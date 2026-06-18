#!/usr/bin/env python3
"""
Launch file for voice-controlled Kobuki robot
Starts: voice recognition → command interpreter → Kobuki driver
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for voice-controlled Kobuki"""
    
    return LaunchDescription([
        # Voice-to-text node (speech recognition)
        Node(
            package='voice_control_logic',
            executable='voice_to_text',
            name='voice_to_text_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # Voice command interpreter (speech → commands)
        Node(
            package='voice_control_logic',
            executable='voice_command_interpreter',
            name='voice_command_interpreter_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # Simple Kobuki driver (commands → robot movement)
        Node(
            package='voice_control_logic',
            executable='simple_kobuki_driver',
            name='simple_kobuki_driver_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
