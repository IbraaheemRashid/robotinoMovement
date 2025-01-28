import pytest
import math
from robotino_hal.nodes.odometry_monitor import RobotinoOdometryMonitor
import rclpy
from geometry_msgs.msg import Quaternion

def test_node_parameters():
    """Test node parameter initialization"""
    rclpy.init()
    node = RobotinoOdometryMonitor()
    
    assert node.frame_id == 'odom'
    assert node.child_frame_id == 'base_link'
    assert node.covariance_linear == 0.1
    assert node.covariance_angular == 0.1
    
    node.destroy_node()
    rclpy.shutdown()

def test_euler_to_quaternion():
    """Test Euler angle to quaternion conversion"""
    rclpy.init()
    node = RobotinoOdometryMonitor()
    
    # Test zero rotation
    quat = node.euler_to_quaternion(0.0)
    assert isinstance(quat, Quaternion)
    assert abs(quat.w - 1.0) < 0.01
    assert abs(quat.x) < 0.01
    assert abs(quat.y) < 0.01
    assert abs(quat.z) < 0.01
    
    # Test 90 degrees rotation
    quat = node.euler_to_quaternion(math.pi/2)
    assert abs(quat.w - 0.707) < 0.01  # cos(pi/4)
    assert abs(quat.z - 0.707) < 0.01  # sin(pi/4)
    
    # Test 180 degrees rotation
    quat = node.euler_to_quaternion(math.pi)
    assert abs(quat.w) < 0.01
    assert abs(quat.z - 1.0) < 0.01
    
    node.destroy_node()
    rclpy.shutdown()

def test_create_odometry_message():
    """Test odometry message creation"""
    rclpy.init()
    node = RobotinoOdometryMonitor()
    
    # Test data [x, y, phi, vx, vy, omega, seq]
    test_data = [1.0, 2.0, 0.5, 0.1, 0.2, 0.3, 1000]
    
    odom_msg, transform_msg = node.create_odometry_message(test_data)
    
    # Check odometry message
    assert odom_msg.header.frame_id == node.frame_id
    assert odom_msg.child_frame_id == node.child_frame_id
    assert abs(odom_msg.pose.pose.position.x - 1.0) < 0.01
    assert abs(odom_msg.pose.pose.position.y - 2.0) < 0.01
    assert abs(odom_msg.twist.twist.linear.x - 0.1) < 0.01
    assert abs(odom_msg.twist.twist.linear.y - 0.2) < 0.01
    assert abs(odom_msg.twist.twist.angular.z - 0.3) < 0.01
    
    # Check transform message
    assert transform_msg.header.frame_id == node.frame_id
    assert transform_msg.child_frame_id == node.child_frame_id
    assert abs(transform_msg.transform.translation.x - 1.0) < 0.01
    assert abs(transform_msg.transform.translation.y - 2.0) < 0.01
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    pytest.main(['-v', __file__])