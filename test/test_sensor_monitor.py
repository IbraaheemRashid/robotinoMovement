#!/usr/bin/env python3

import pytest
from std_msgs.msg import Float32MultiArray
from robotino_hal.nodes.sensor_monitor import RobotinoSensorMonitor
from .test_base import RobotinoNodeTestBase

class TestSensorMonitor(RobotinoNodeTestBase):
    """Test cases for sensor monitor node"""
    
    def get_node_name(self):
        return 'sensor_monitor'
    
    @pytest.fixture(autouse=True)
    def setup_method(self):
        """Setup before each test"""
        self.create_subscription('robotino/distancesensorarray', Float32MultiArray)
    
    def test_sensor_publishing(self):
        """Test that sensor monitor publishes messages"""
        assert self.wait_for_message('robotino/distancesensorarray'), \
            "No sensor message received"
        
        msg = self.get_last_message('robotino/distancesensorarray')
        assert msg is not None, "No message in queue"
        assert len(msg.data) == 9, "Should have 9 sensor readings"
        assert all(x >= 0.0 for x in msg.data), "All readings should be non-negative"
    
    def test_sensor_update_rate(self):
        """Test that sensor monitor publishes at correct rate"""
        self.verify_publishing_rate('robotino/distancesensorarray', 10.0)
    
    def test_sensor_unit_conversion(self):
        """Test that sensor values are converted from mm to meters"""
        # Set mock response in millimeters
        self.mock_responses['/data/distancesensorarray'] = [1000.0] * 9
        
        assert self.wait_for_message('robotino/distancesensorarray'), \
            "No sensor message received"
        
        msg = self.get_last_message('robotino/distancesensorarray')
        assert all(abs(x - 1.0) < 0.001 for x in msg.data), \
            "Sensor values should be converted to meters"
    
    def test_api_error_handling(self):
        """Test handling of API errors"""
        # Simulate API error
        def raise_error(*args, **kwargs):
            raise Exception("API Error")
        
        self.mock_get.side_effect = raise_error
        
        # Should continue running but log error
        assert self.wait_for_message('robotino/distancesensorarray', timeout=2.0) is False, \
            "Should not receive messages during API error"
    
    def test_invalid_sensor_data(self):
        """Test handling of invalid sensor data"""
        # Test with wrong number of sensors
        self.mock_responses['/data/distancesensorarray'] = [200.0] * 8  # Only 8 sensors
        
        assert self.wait_for_message('robotino/distancesensorarray', timeout=2.0) is False, \
            "Should not publish invalid sensor data"