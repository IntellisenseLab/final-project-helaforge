#!/usr/bin/env python3
"""
Simple Kobuki Driver - Python-based serial communication
Bypasses complex C++ ROS 2 dependencies
Communicates directly with Kobuki over /dev/ttyUSB0
"""

import serial
import time
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleKobukiDriver(Node):
    """ROS 2 node that controls Kobuki robot via serial port"""
    
    # Kobuki protocol values
    HEADER1 = 0xAA
    HEADER2 = 0x55
    CMD_BASE_CONTROL = 0x01  # Base motor control command
    CMD_BASE_STOP = 0x00     # Stop command
    
    def __init__(self):
        super().__init__('simple_kobuki_driver')
        
        # Serial port setup
        self.ser = None
        self.connected = False
        
        # Motor parameters (mm/s and rad/s)
        self.max_linear_vel = 500.0   # mm/s
        self.max_angular_vel = 2.84   # rad/s (180 deg/s)
        self.wheel_separation = 230.0 # mm between wheels
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.connected = True
            self.get_logger().info('✓ Connected to Kobuki on /dev/ttyUSB0')
            time.sleep(1)  # Let the connection stabilize
        except serial.SerialException as e:
            self.get_logger().error(f'✗ Failed to connect to Kobuki: {e}')
            self.get_logger().warn('Make sure: 1) Kobuki is plugged in, 2) You ran: sudo usermod -a -G dialout $USER')
        
        # Subscribe to cmd_vel topic (standard ROS 2 velocity command)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscribed to /cmd_vel topic')
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        if not self.connected:
            return
        
        linear_vel = msg.linear.x      # m/s, positive = forward
        angular_vel = msg.angular.z    # rad/s, positive = counter-clockwise
        
        # Convert to mm/s for Kobuki
        linear_mm_s = linear_vel * 1000.0
        
        # Clamp velocities to safe limits
        linear_mm_s = max(-self.max_linear_vel, min(self.max_linear_vel, linear_mm_s))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # Send command to Kobuki
        self.send_velocity_command(linear_mm_s, angular_vel)
        
        log_msg = f'Cmd: linear={linear_vel:.2f}m/s, angular={angular_vel:.2f}rad/s'
        self.get_logger().info(log_msg)
    
    def send_velocity_command(self, linear_vel_mm_s, angular_vel_rad_s):
        """
        Send velocity command to Kobuki
        Uses differential drive kinematics to convert linear/angular to left/right wheel velocities
        """
        if not self.connected or self.ser is None:
            return
        
        try:
            # Differential drive kinematics
            # v_left = (v_linear - w_angular * L/2)
            # v_right = (v_linear + w_angular * L/2)
            half_separation = self.wheel_separation / 2.0
            
            left_vel = linear_vel_mm_s - (angular_vel_rad_s * half_separation)
            right_vel = linear_vel_mm_s + (angular_vel_rad_s * half_separation)
            
            # Clamp wheel velocities
            left_vel = int(max(-self.max_linear_vel, min(self.max_linear_vel, left_vel)))
            right_vel = int(max(-self.max_linear_vel, min(self.max_linear_vel, right_vel)))
            
            # Kobuki motor command format: [HEADER1, HEADER2, LENGTH, CMD, DATA..., CHECKSUM]
            # Base control format: Cmd=0x01, then 2 bytes each for left and right motor velocity
            length = 4  # length of the command payload
            cmd = self.CMD_BASE_CONTROL
            
            # Encode velocities as 16-bit signed integers (little-endian)
            payload = struct.pack('<hh', left_vel, right_vel)
            data = bytes([cmd]) + payload
            
            # Calculate checksum (XOR of all bytes in data)
            checksum = 0
            for byte in data:
                checksum ^= byte
            
            # Build complete message
            message = bytes([self.HEADER1, self.HEADER2, length]) + data + bytes([checksum])
            
            self.ser.write(message)
            
        except Exception as e:
            self.get_logger().error(f'Error sending velocity command: {e}')
            self.connected = False
    
    def send_stop_command(self):
        """Stop robot movement"""
        if self.connected:
            self.send_velocity_command(0, 0)


def main(args=None):
    rclpy.init(args=args)
    
    driver = SimpleKobukiDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info('Shutting down...')
    finally:
        if driver.ser and driver.ser.is_open:
            driver.send_stop_command()
            driver.ser.close()
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
