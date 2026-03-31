#!/usr/bin/env python3
"""
Voice Command Interpreter
Listens to recognized speech and converts to Kobuki velocity commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class VoiceCommandInterpreter(Node):
    """Converts voice commands to Kobuki movement commands"""
    
    def __init__(self):
        super().__init__('voice_command_interpreter')
        
        # Subscribe to recognized speech
        self.speech_subscription = self.create_subscription(
            String,
            '/recognized_speech',
            self.speech_callback,
            10
        )
        
        # Publish velocity commands to Kobuki
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Command parameters
        self.linear_speed = 0.3       # m/s
        self.angular_speed = 1.0      # rad/s
        self.stop_timeout = 2.0       # seconds before auto-stopping
        
        self.get_logger().info('Voice Command Interpreter started')
        self.get_logger().info('Available commands: forward, backward, left, right, stop, spin')
    
    def speech_callback(self, msg: String):
        """Process recognized speech and convert to movement commands"""
        text = msg.data.lower().strip()
        self.get_logger().info(f'Processing command: "{text}"')
        
        # Parse command and send velocity
        twist = Twist()
        executed = False
        
        # Forward commands
        if any(word in text for word in ['forward', 'go', 'move forward', 'ahead', 'front']):
            twist.linear.x = self.linear_speed
            self.get_logger().info('→ Moving FORWARD')
            executed = True
        
        # Backward commands
        elif any(word in text for word in ['backward', 'back', 'reverse', 'behind']):
            twist.linear.x = -self.linear_speed
            self.get_logger().info('← Moving BACKWARD')
            executed = True
        
        # Left turn commands
        elif any(word in text for word in ['left', 'turn left', 'left turn']):
            twist.angular.z = self.angular_speed
            self.get_logger().info('↺ Turning LEFT')
            executed = True
        
        # Right turn commands
        elif any(word in text for word in ['right', 'turn right', 'right turn']):
            twist.angular.z = -self.angular_speed
            self.get_logger().info('↻ Turning RIGHT')
            executed = True
        
        # Spin/rotate commands
        elif any(word in text for word in ['spin', 'rotate', 'turn around']):
            twist.angular.z = self.angular_speed * 1.5
            self.get_logger().info('⟲ SPINNING')
            executed = True
        
        # Stop commands
        elif any(word in text for word in ['stop', 'halt', 'pause', 'stay']):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('⊘ STOP')
            executed = True
        
        # Send command if recognized
        if executed:
            self.cmd_vel_publisher.publish(twist)
        else:
            self.get_logger().warn(f'Unknown command: "{text}"')
    
    def send_stop_command(self):
        """Emergency stop"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    interpreter = VoiceCommandInterpreter()
    
    try:
        rclpy.spin(interpreter)
    except KeyboardInterrupt:
        interpreter.get_logger().info('Shutting down...')
    finally:
        interpreter.send_stop_command()
        interpreter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
