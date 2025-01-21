#!/usr/bin/env python3

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robotino_hal.nodes.motor_control import RobotinoMotorController
from .test_base import RobotinoNodeTestBase
import time

class TestMotorControl(RobotinoNodeTestBase):
    """Test cases for motor control node"""
    
    def get_node_name(self):
        return 'motor_control'
    
    def setup_method(self):
        """Setup publishers for each test"""
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
    
    def test_velocity_command(self):
        """Test basic velocity command handling"""
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 0.2
        
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.1)  # Allow time for command processing
        
        # Verify API call
        self.mock_post.assert_called_with(
            'http://localhost:8081/data/omnidrive',
            json=[0.5, 0.0, 0.2]
        )
    
    def test_velocity_limits(self):
        """Test velocity limiting"""
        cmd = Twist()
        cmd.linear.x = 2.0  # Above max
        cmd.angular.z = 3.0  # Above max
        
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.1)
        
        # Verify command was limited
        self.mock_post.assert_called()
        args = self.mock_post.call_args[1]['json']
        assert abs(args[0]) <= 0.8, "Linear velocity should be limited"
        assert abs(args[2]) <= 1.5, "Angular velocity should be limited"
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        # Send movement command
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.1)
        
        # Activate emergency stop
        estop = Bool()
        estop.data = True
        self.emergency_stop_pub.publish(estop)
        time.sleep(0.1)
        
        # Verify stop command sent
        self.mock_post.assert_called_with(
            'http://localhost:8081/data/omnidrive',
            json=[0.0, 0.0, 0.0]
        )
        
        # Try to send command during e-stop
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.1)
        
        # Should not send command during e-stop
        assert self.mock_post.call_count == 2, "Should not send commands during e-stop"
    
    def test_acceleration_limiting(self):
        """Test acceleration limiting"""
        cmd = Twist()
        cmd.linear.x = 0.5
        
        # Send rapid commands
        for _ in range(5):
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.02)  # 50Hz
        
        # Get all velocity commands
        calls = [call[1]['json'][0] for call in self.mock_post.call_args_list]
        
        # Check that velocity changes are limited
        max_change = max(abs(calls[i] - calls[i-1]) for i in range(1, len(calls)))
        assert max_change <= 0.025, "Acceleration should be limited"  # 0.5 m/s² * 0.02s = 0.025
    
    def test_api_error_handling(self):
        """Test handling of API errors"""
        def raise_error(*args, **kwargs):
            raise Exception("API Error")
        
        self.mock_post.side_effect = raise_error
        
        # Send command
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.1)
        
        # Should handle error gracefully
        assert self.mock_post.called, "Should attempt to send command"