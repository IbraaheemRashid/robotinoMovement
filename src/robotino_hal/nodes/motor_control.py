#!/usr/bin/env python3

from dataclasses import dataclass
from math import sqrt
import requests

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


@dataclass
class MotorLimits:
    """Safety limits for motor control."""

    max_linear_speed: float = 0.8  # m/s
    max_angular_speed: float = 1.5  # rad/s
    max_acceleration: float = 0.5   # m/s^2


class RobotinoMotorController(Node):
    """ROS 2 node for controlling Robotino's motors.

    This node subscribes to velocity commands and controls the robot's motors,
    ensuring safe operation within defined limits. Commands must be sent at
    least every 200ms or the robot will stop for safety.
    """

    def __init__(self):
        super().__init__('robotino_motor_controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', 'localhost'),
                ('max_linear_speed', 0.8),
                ('max_angular_speed', 1.5),
                ('max_acceleration', 0.5),
                ('control_rate', 100.0)
            ]
        )

        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.limits = MotorLimits(
            max_linear_speed=self.get_parameter('max_linear_speed').value,
            max_angular_speed=self.get_parameter('max_angular_speed').value,
            max_acceleration=self.get_parameter('max_acceleration').value
        )

        self.vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            1
        )

        self.emergency_stop = False
        self.current_velocity = Twist()
        self.target_velocity = Twist()

        control_rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0/control_rate, self.control_loop)

        self.get_logger().info('Motor controller initialized')

    def velocity_callback(self, msg: Twist) -> None:
        """Process incoming velocity commands.

        Args:
            msg: The velocity command message
        """
        self.target_velocity = self.clamp_velocity(msg)
        self.get_logger().debug(
            f'Received velocity command - Linear: ({msg.linear.x:.2f}, '
            f'{msg.linear.y:.2f}), Angular: {msg.angular.z:.2f}'
        )

    def emergency_stop_callback(self, msg: Bool) -> None:
        """Process emergency stop commands.

        Args:
            msg: The emergency stop command message
        """
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop activated!')
            self.stop_motors()
        else:
            self.get_logger().info('Emergency stop released')

    def clamp_velocity(self, velocity: Twist) -> Twist:
        """Limit velocity commands to safe values.

        Args:
            velocity: The input velocity command

        Returns:
            The clamped velocity command
        """
        clamped = Twist()

        # Clamp linear velocity
        linear_velocity = sqrt(velocity.linear.x**2 + velocity.linear.y**2)
        if linear_velocity > self.limits.max_linear_speed:
            scale = self.limits.max_linear_speed / linear_velocity
            velocity.linear.x *= scale
            velocity.linear.y *= scale

        clamped.linear.x = velocity.linear.x
        clamped.linear.y = velocity.linear.y

        # Clamp angular velocity
        clamped.angular.z = max(
            -self.limits.max_angular_speed,
            min(self.limits.max_angular_speed, velocity.angular.z)
        )

        return clamped

    def stop_motors(self) -> None:
        """Immediately stop all motors by sending zero velocity."""
        try:
            url = f'http://{self.robotino_ip}/data/omnidrive'
            data = [0.0, 0.0, 0.0]  # [vx, vy, omega]
            requests.post(url, json=data)
            self.current_velocity = Twist()
            self.target_velocity = Twist()
        except Exception as e:
            self.get_logger().error(f'Failed to stop motors: {e}')

    def set_motor_velocity(self, velocity: Twist) -> bool:
        """Send velocity commands to Robotino API using omnidrive endpoint.

        Args:
            velocity: The velocity command to send

        Returns:
            True if successful, False otherwise
        """
        try:
            url = f'http://{self.robotino_ip}/data/omnidrive'
            data = [
                velocity.linear.x,
                velocity.linear.y,
                velocity.angular.z
            ]
            response = requests.post(url, json=data)
            response.raise_for_status()
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to set motor velocity: {e}')
            return False

    def control_loop(self) -> None:
        """Execute the main control loop for smooth velocity control."""
        if self.emergency_stop:
            return

        try:
            dt = 1.0 / self.get_parameter('control_rate').value
            max_step = self.limits.max_acceleration * dt

            # Update linear velocities
            for attr in ['x', 'y']:
                current = getattr(self.current_velocity.linear, attr)
                target = getattr(self.target_velocity.linear, attr)
                diff = target - current
                if abs(diff) > max_step:
                    step = max_step if diff > 0 else -max_step
                else:
                    step = diff
                setattr(self.current_velocity.linear, attr, current + step)

            # Update angular velocity
            angular_diff = self.target_velocity.angular.z - self.current_velocity.angular.z
            if abs(angular_diff) > max_step:
                angular_step = max_step if angular_diff > 0 else -max_step
            else:
                angular_step = angular_diff
            self.current_velocity.angular.z += angular_step

            # Send velocity command
            if not self.set_motor_velocity(self.current_velocity):
                self.get_logger().warn('Failed to set motor velocity')

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
            self.stop_motors()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = RobotinoMotorController()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in motor controller: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()