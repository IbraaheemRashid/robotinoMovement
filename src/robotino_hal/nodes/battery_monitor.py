#!/usr/bin/env python3

import json

from dataclasses import dataclass

import requests

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


@dataclass
class BatteryConfig:
    """Configuration parameters for battery monitoring."""

    min_voltage: float = 22.0
    max_voltage: float = 24.5
    voltage_scale: float = 350.0
    critical_percentage: float = 0.15  # 15%
    warning_percentage: float = 0.25   # 25%


class RobotinoBatteryMonitor(Node):
    """ROS 2 node for monitoring Robotino's battery state.

    This node periodically fetches battery data from the Robotino API and publishes
    the battery state including voltage and computed percentage.
    """

    def __init__(self):
        super().__init__('robotino_battery_monitor')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robotino_ip', 'localhost'),
                ('update_rate', 1.0),  # Hz
                ('timeout', 1.0)       # seconds
            ]
        )

        # Get parameters
        self.robotino_ip = self.get_parameter('robotino_ip').value
        self.update_rate = self.get_parameter('update_rate').value
        self.timeout = self.get_parameter('timeout').value

        # Battery configuration
        self.battery_config = BatteryConfig()

        # QoS profile for battery data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publisher
        self.battery_publisher = self.create_publisher(
            BatteryState,
            'robotino/battery_state',
            qos_profile
        )

        # Create timer for periodic updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_battery_state
        )

        # Diagnostic data
        self.consecutive_errors = 0
        self.MAX_CONSECUTIVE_ERRORS = 5
        self.last_voltage: Optional[float] = None

        self.get_logger().info('Battery monitor initialized')

    def get_battery_data(self) -> Optional[float]:
        """Fetch battery data from Robotino API.

        Returns:
            Optional[float]: Battery voltage if successful, None otherwise
        """
        try:
            url = f'http://{self.robotino_ip}/data/powermanagement'
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()

            data = json.loads(response.text)
            raw_voltage = float(data['voltage'])
            actual_voltage = raw_voltage * self.battery_config.voltage_scale

            self.consecutive_errors = 0  # Reset error counter
            return actual_voltage

        except requests.Timeout:
            self.get_logger().warn('Timeout while fetching battery data')
        except requests.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {e}')
        except (ValueError, KeyError) as e:
            self.get_logger().error(f'Failed to parse battery data: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

        self.consecutive_errors += 1
        if self.consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
            self.get_logger().error(
                'Multiple consecutive errors. Check robot connection!'
            )
        return None

    def calculate_battery_percentage(self, voltage: float) -> float:
        """Calculate battery percentage based on voltage.

        Args:
            voltage (float): Current battery voltage

        Returns:
            float: Battery percentage between 0.0 and 1.0
        """
        percentage = (voltage - self.battery_config.min_voltage) / (
            self.battery_config.max_voltage - self.battery_config.min_voltage
        )
        return max(0.0, min(1.0, percentage))

    def check_battery_health(self, percentage: float) -> None:
        """Check battery health and log warnings if needed.

        Args:
            percentage (float): Current battery percentage between 0.0 and 1.0
        """
        if percentage <= self.battery_config.critical_percentage:
            self.get_logger().error(
                f'CRITICAL BATTERY LEVEL: {percentage*100:.1f}%!'
            )
        elif percentage <= self.battery_config.warning_percentage:
            self.get_logger().warn(
                f'Low battery warning: {percentage*100:.1f}%'
            )

    def create_battery_message(self, voltage: float) -> BatteryState:
        """Create a BatteryState message.

        Args:
            voltage (float): Current battery voltage

        Returns:
            BatteryState: Populated battery state message
        """
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.present = True

        msg.percentage = self.calculate_battery_percentage(voltage)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Set health based on percentage
        if msg.percentage <= self.battery_config.critical_percentage:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_CRITICAL
        elif msg.percentage <= self.battery_config.warning_percentage:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEGRADED
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        return msg

    def publish_battery_state(self) -> None:
        """Publish current battery state."""
        voltage = self.get_battery_data()

        if voltage is None:
            return

        # Check for unrealistic voltage changes
        if self.last_voltage is not None:
            voltage_change = abs(voltage - self.last_voltage)
            if voltage_change > 1.0:  # More than 1V change
                self.get_logger().warn(
                    f'Unusual voltage change detected: {voltage_change:.2f}V'
                )

        self.last_voltage = voltage

        try:
            msg = self.create_battery_message(voltage)
            self.check_battery_health(msg.percentage)
            self.battery_publisher.publish(msg)

            self.get_logger().info(
                f'Battery voltage: {voltage:.2f}V, '
                f'Percentage: {msg.percentage*100:.1f}%'
            )

        except Exception as e:
            self.get_logger().error(f'Error publishing battery state: {e}')


def main(args=None):
    """Starts Main entry point."""
    rclpy.init(args=args)

    try:
        node = RobotinoBatteryMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in battery monitor: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    