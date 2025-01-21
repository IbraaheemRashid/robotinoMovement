#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
from threading import Event

class TestNodeInteractions:
    """Integration tests for Robotino HAL nodes"""
    
    @pytest.fixture(autouse=True)
    def setup(self):
        """Setup test environment"""
        rclpy.init()
        self.node = rclpy.create_node('test_integration')
        
        # Publishers
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.emergency_stop_pub = self.node.create_publisher(
            Bool,
            'emergency_stop',
            10
        )
        
        # Track motor commands sent to mock server
        self.motor_commands = []
        self.command_received = Event()
        
        yield
        
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        # First send a movement command
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)  # Wait for command to be processed
        
        # Activate emergency stop
        estop_msg = Bool()
        estop_msg.data = True
        self.emergency_stop_pub.publish(estop_msg)
        time.sleep(0.5)  # Wait for e-stop to be processed
        
        # Try to send movement command during e-stop
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)
        
        # Release emergency stop
        estop_msg.data = False
        self.emergency_stop_pub.publish(estop_msg)
        time.sleep(0.5)
        
        # Movement should work again
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)
    
    def test_velocity_limits(self):
        """Test that velocity commands are properly limited"""
        cmd = Twist()
        
        # Try excessive linear velocity
        cmd.linear.x = 2.0  # Should be limited to 0.8
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)
        
        # Try excessive angular velocity
        cmd.linear.x = 0.0
        cmd.angular.z = 3.0  # Should be limited to 1.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.5)