#!/usr/bin/env python3

import pytest
from sensor_msgs.msg import BatteryState
from robotino_hal.nodes.battery_monitor import RobotinoBatteryMonitor
from .test_base import RobotinoNodeTestBase

class TestBatteryMonitor(RobotinoNodeTestBase):
    """Test cases for battery monitor node"""
    
    def get_node_name(self):
        return 'battery_monitor'
    
    @pytest.fixture(autouse=True)
    def setup_method(self):
        """Setup before each test"""
        self.create_subscription('robotino/battery_state', BatteryState)
    
    def test_battery_publishing(self):
        """Test that battery monitor publishes messages"""
        # Verify we receive messages
        assert self.wait_for_message('robotino/battery_state'), \
            "No battery message received"
        
        # Check message content
        msg = self.get_last_message('robotino/battery_state')
        assert msg is not None, "No message in queue"
        assert msg.voltage > 0, "Battery voltage should be positive"
        assert 0 <= msg.percentage <= 1.0, "Battery percentage should be between 0 and 1"
        assert msg.present, "Battery should be marked as present"
    
    def test_battery_voltage_range(self):
        """Test that battery voltage stays within expected range"""
        assert self.wait_for_message('robotino/battery_state'), \
            "No battery message received"
        
        msg = self.get_last_message('robotino/battery_state')
        assert 22.0 <= msg.voltage <= 24.5, \
            f"Voltage {msg.voltage}V outside expected range"
    
    def test_battery_update_rate(self):
        """Test that battery monitor publishes at correct rate"""
        self.verify_publishing_rate('robotino/battery_state', 1.0)
    
    def test_low_battery_warning(self):
        """Test low battery warning thresholds"""
        # Simulate low battery
        self.mock_responses['/data/powermanagement']['voltage'] = 22.5/350.0
        
        assert self.wait_for_message('robotino/battery_state'), \
            "No battery message received"
        
        msg = self.get_last_message('robotino/battery_state')
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEGRADED, \
            "Battery should be in degraded state"
    
    def test_critical_battery(self):
        """Test critical battery threshold"""
        # Simulate critical battery
        self.mock_responses['/data/powermanagement']['voltage'] = 22.1/350.0
        
        assert self.wait_for_message('robotino/battery_state'), \
            "No battery message received"
        
        msg = self.get_last_message('robotino/battery_state')
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_CRITICAL, \
            "Battery should be in critical state"
    
    def test_api_error_handling(self):
        """Test handling of API errors"""
        # Simulate API error
        def raise_error(*args, **kwargs):
            raise Exception("API Error")
        
        self.mock_get.side_effect = raise_error
        
        # Should continue running but log error
        assert self.wait_for_message('robotino/battery_state', timeout=2.0) is False, \
            "Should not receive messages during API error"