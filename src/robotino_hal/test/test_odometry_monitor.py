#!/usr/bin/env python3

import pytest
from nav_msgs.msg import Odometry
from robotino_hal.nodes.odometry_monitor import RobotinoOdometryMonitor
from .test_base import RobotinoNodeTestBase
import math
import time

class TestOdometryMonitor(RobotinoNodeTestBase):
    """Test cases for odometry monitor node"""
    
    def get_node_name(self):
        return 'odometry_monitor'
    
    @pytest.fixture(autouse=True)
    def setup_method(self):
        """Setup before each test"""
        self.create_subscription('robotino/odom', Odometry)
    
    def test_odometry_publishing(self):
        """Test that odometry monitor publishes messages"""
        assert self.wait_for_message('robotino/odom'), \
            "No odometry message received"
        
        msg = self.get_last_message('robotino/odom')
        assert msg is not None, "No message in queue"
        assert msg.header.frame_id == "odom", "Wrong frame_id"
        assert msg.child_frame_id == "base_link", "Wrong child_frame_id"
    
    def test_odometry_update_rate(self):
        """Test that odometry monitor publishes at correct rate"""
        self.verify_publishing_rate('robotino/odom', 20.0)
    
    def test_stationary_robot(self):
        """Test odometry when robot is stationary"""
        self.mock_responses['/data/odometry'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, int(time.time()*1000)]
        
        assert self.wait_for_message('robotino/odom'), \
            "No odometry message received"
        
        msg = self.get_last_message('robotino/odom')
        assert abs(msg.twist.twist.linear.x) < 0.001, "Should have zero velocity"
        assert abs(msg.twist.twist.angular.z) < 0.001, "Should have zero angular velocity"
    
    def test_moving_robot(self):
        """Test odometry when robot is moving"""
        self.mock_responses['/data/odometry'] = [1.0, 2.0, math.pi/4, 0.5, 0.0, 0.2, int(time.time()*1000)]
        
        assert self.wait_for_message('robotino/odom'), \
            "No odometry message received"
        
        msg = self.get_last_message('robotino/odom')
        assert abs(msg.pose.pose.position.x - 1.0) < 0.001, "Wrong x position"
        assert abs(msg.pose.pose.position.y - 2.0) < 0.001, "Wrong y position"
        assert abs(msg.twist.twist.linear.x - 0.5) < 0.001, "Wrong linear velocity"
        assert abs(msg.twist.twist.angular.z - 0.2) < 0.001, "Wrong angular velocity"
    
    def test_orientation_conversion(self):
        """Test conversion from euler to quaternion"""
        # Test different angles
        test_angles = [0, math.pi/4, math.pi/2, math.pi, -math.pi/2]
        
        for angle in test_angles:
            self.mock_responses['/data/odometry'] = [0.0, 0.0, angle, 0.0, 0.0, 0.0, int(time.time()*1000)]
            
            assert self.wait_for_message('robotino/odom'), \
                "No odometry message received"
            
            msg = self.get_last_message('robotino/odom')
            quat = msg.pose.pose.orientation
            
            # Convert quaternion back to yaw
            siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            assert abs(yaw - angle) < 0.001, f"Wrong orientation conversion for angle {angle}"
    
    def test_covariance_matrix(self):
        """Test covariance matrix values"""
        assert self.wait_for_message('robotino/odom'), \
            "No odometry message received"
        
        msg = self.get_last_message('robotino/odom')
        
        # Check diagonal elements (should be non-zero)
        assert msg.pose.covariance[0] > 0, "Zero position covariance"
        assert msg.pose.covariance[7] > 0, "Zero position covariance"
        assert msg.pose.covariance[35] > 0, "Zero orientation covariance"
    
    def test_api_error_handling(self):
        """Test handling of API errors"""
        def raise_error(*args, **kwargs):
            raise Exception("API Error")
        
        self.mock_get.side_effect = raise_error
        
        # Should continue running but log error
        assert self.wait_for_message('robotino/odom', timeout=2.0) is False, \
            "Should not receive messages during API error"