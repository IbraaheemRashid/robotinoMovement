#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
from tf2_ros import Buffer, TransformListener
import numpy as np
from rclpy.logging import LoggingSeverity

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('navigation')
        
        # Set logger level to DEBUG
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        
        # Parameters
        self.declare_parameter('navigation.max_linear_speed', 0.5)
        self.declare_parameter('navigation.max_angular_speed', 1.0)
        self.declare_parameter('navigation.position_tolerance', 0.1)
        self.declare_parameter('navigation.angular_tolerance', 0.35)
        
        self.max_linear_speed = self.get_parameter('navigation.max_linear_speed').value
        self.max_angular_speed = self.get_parameter('navigation.max_angular_speed').value
        self.position_tolerance = self.get_parameter('navigation.position_tolerance').value
        self.angular_tolerance = self.get_parameter('navigation.angular_tolerance').value
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            'goal_pose',
            self.goal_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'robotino/odom',
            self.odom_callback,
            10
        )
        
        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.is_active = False
        
        # Control loop timer (10Hz)
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('=== Navigation Node Started ===')
        self.get_logger().info(f'Parameters loaded:')
        self.get_logger().info(f'  max_linear_speed: {self.max_linear_speed}')
        self.get_logger().info(f'  max_angular_speed: {self.max_angular_speed}')
        self.get_logger().info(f'  position_tolerance: {self.position_tolerance}')
        self.get_logger().info(f'  angular_tolerance: {self.angular_tolerance}')

    def goal_callback(self, msg: PoseStamped):
        """Handle new navigation goals"""
        self.goal_pose = msg.pose
        self.is_active = True
        self.get_logger().info('\n=== New Goal Received ===')
        self.get_logger().info(f'Goal position: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}')
        if self.current_pose:
            self.get_logger().info(f'Current position: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}')
        else:
            self.get_logger().warn('No current pose available!')

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        if self.current_pose is None:
            self.get_logger().info('First odometry message received')
        self.current_pose = msg.pose.pose

    def calculate_distance_to_goal(self):
        """Calculate distance and angle to current goal"""
        if not self.current_pose or not self.goal_pose:
            return None, None
            
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        return distance, angle

    def get_yaw_from_pose(self, pose):
        """Extract yaw angle from pose quaternion"""
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop for navigation"""
        if not self.is_active:
            return
            
        if not self.current_pose:
            self.get_logger().warn('Waiting for odometry data...')
            return
            
        if not self.goal_pose:
            self.get_logger().warn('No goal set')
            return
            
        # Calculate distance and angle to goal
        distance, target_angle = self.calculate_distance_to_goal()
        if distance is None:
            self.get_logger().error('Failed to calculate distance to goal')
            return
            
        # Get current heading
        current_angle = self.get_yaw_from_pose(self.current_pose)
        
        # Calculate angle difference
        angle_diff = target_angle - current_angle
        # Normalize to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        self.get_logger().info(f'\n=== Navigation Status ===')
        self.get_logger().info(f'Current: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}')
        self.get_logger().info(f'Distance to goal: {distance:.3f}m')
        self.get_logger().info(f'Angle difference: {math.degrees(angle_diff):.1f}°')
        
        # Check if we've reached the goal
        if distance < self.position_tolerance and abs(angle_diff) < self.angular_tolerance:
            self.is_active = False
            self.stop_robot()
            self.get_logger().info('=== Goal Reached! ===')
            return
            
        # Create velocity command
        cmd = Twist()
        
        # First, rotate to face the goal
        if abs(angle_diff) > self.angular_tolerance:
            # Pure rotation
            cmd.angular.z = self.max_angular_speed * np.sign(angle_diff)
            self.get_logger().info(f'Rotating with speed: {cmd.angular.z:.2f}')
        else:
            # Move towards goal once oriented
            cmd.linear.x = min(self.max_linear_speed, distance)
            # Keep orientation correct while moving
            cmd.angular.z = self.max_angular_speed * angle_diff
            self.get_logger().info(f'Moving: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')
            
        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot by publishing zero velocity"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()