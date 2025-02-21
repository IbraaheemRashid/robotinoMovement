#!/usr/bin/env python3

from math import atan2, cos, degrees, sin, sqrt

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class SimpleNavigator(Node):
    """A ROS 2 node for basic robot navigation.

    This node implements simple goal-based navigation using position and orientation
    control. It subscribes to goal poses and odometry, and publishes velocity
    commands to drive the robot to its target.
    """

    def __init__(self):
        super().__init__('navigation')

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
            'odom',
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
        self.get_logger().info('Parameters loaded:')
        self.get_logger().info(f'  max_linear_speed: {self.max_linear_speed}')
        self.get_logger().info(f'  max_angular_speed: {self.max_angular_speed}')
        self.get_logger().info(f'  position_tolerance: {self.position_tolerance}')
        self.get_logger().info(f'  angular_tolerance: {self.angular_tolerance}')

    def goal_callback(self, msg: PoseStamped) -> None:
        """Handle new navigation goals.

        Args:
            msg: The goal pose message
        """
        self.goal_pose = msg.pose
        self.is_active = True

        # Log new goal information
        log = self.get_logger()
        log.info('=== New Goal Received ===')
        log.info('Goal position: '
                f'x={msg.pose.position.x:.3f}, '
                f'y={msg.pose.position.y:.3f}')

        if self.current_pose:
            log.info('Current position: '
                    f'x={self.current_pose.position.x:.3f}, '
                    f'y={self.current_pose.position.y:.3f}')
        else:
            log.warn('No current pose available!')

    def odom_callback(self, msg: Odometry) -> None:
        """Handle odometry updates.

        Args:
            msg: The odometry message
        """
        if self.current_pose is None:
            self.get_logger().info('First odometry message received')
        self.current_pose = msg.pose.pose

    def calculate_distance_to_goal(self) -> tuple[float | None, float | None]:
        """Calculate distance and angle to current goal.

        Returns:
            A tuple of (distance, angle) to the goal, or (None, None) if not calculable
        """
        if not self.current_pose or not self.goal_pose:
            return None, None

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = sqrt(dx*dx + dy*dy)
        angle = atan2(dy, dx)

        return distance, angle

    def get_yaw_from_pose(self, pose: PoseStamped) -> float:
        """Extract yaw angle from pose quaternion.

        Args:
            pose: The pose containing orientation quaternion

        Returns:
            The yaw angle in radians
        """
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return atan2(siny_cosp, cosy_cosp)

    def control_loop(self) -> None:
        """Execute the main navigation control loop.

        This method implements the core navigation logic, calculating and
        publishing velocity commands to move the robot toward its goal.
        """
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

        # Calculate angle difference and normalize to [-pi, pi]
        angle_diff = target_angle - current_angle
        angle_diff = atan2(sin(angle_diff), cos(angle_diff))

        # Log navigation status
        log = self.get_logger()
        log.info('=== Navigation Status ===')
        log.info('Current: '
                f'x={self.current_pose.position.x:.3f}, '
                f'y={self.current_pose.position.y:.3f}')
        log.info(f'Distance to goal: {distance:.3f}m')
        log.info(f'Angle difference: {degrees(angle_diff):.1f}°')

        # Check if we've reached the goal
        if (distance < self.position_tolerance and
                abs(angle_diff) < self.angular_tolerance):
            self.is_active = False
            self.stop_robot()
            log.info('=== Goal Reached! ===')
            return

        # Create velocity command
        cmd = Twist()

        # First, rotate to face the goal
        if abs(angle_diff) > self.angular_tolerance:
            # Pure rotation
            cmd.angular.z = self.max_angular_speed * np.sign(angle_diff)
            log.info(f'Rotating with speed: {cmd.angular.z:.2f}')
        else:
            # Move towards goal once oriented
            cmd.linear.x = min(self.max_linear_speed, distance)
            # Keep orientation correct while moving
            cmd.angular.z = self.max_angular_speed * angle_diff
            log.info('Moving: '
                    f'linear={cmd.linear.x:.2f}, '
                    f'angular={cmd.angular.z:.2f}')

        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self) -> None:
        """Stop the robot by publishing zero velocity."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped')


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    node = SimpleNavigator()

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()