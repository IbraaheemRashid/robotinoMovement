#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from threading import Event
import time
import pytest
import requests
import json
from unittest.mock import MagicMock, patch

class RobotinoNodeTestBase:
    """Base class for testing Robotino HAL nodes"""
    
    @pytest.fixture(autouse=True)
    def setup_class(self):
        """Setup test fixture before each test"""
        rclpy.init()
        self.node = rclpy.create_node(f'test_{self.get_node_name()}')
        self.messages = {}
        self.message_received = {}
        self.setup_mock_api()
        yield  
        self.node.destroy_node()
        rclpy.shutdown()
    
    def get_node_name(self):
        """Override this in child class to return node name"""
        raise NotImplementedError
    
    def setup_mock_api(self):
        """Setup mock responses for Robotino API"""
        self.mock_responses = {
            '/data/powermanagement': {'voltage': 24.0/350.0},
            '/data/distancesensorarray': [200.0] * 9,
            '/data/odometry': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, int(time.time()*1000)],
            '/data/omnidrive': {'status': 'ok'}
        }
        
        def mock_get(*args, **kwargs):
            mock_response = MagicMock()
            url = args[0]
            endpoint = url.split('localhost:8081')[-1]
            
            if endpoint not in self.mock_responses:
                mock_response.ok = False
                return mock_response
                
            mock_response.ok = True
            mock_response.text = json.dumps(self.mock_responses[endpoint])
            mock_response.content = mock_response.text.encode()
            return mock_response
            
        def mock_post(*args, **kwargs):
            mock_response = MagicMock()
            mock_response.ok = True
            return mock_response
        
        # Patch requests
        self.requests_patcher = patch('requests.get', side_effect=mock_get)
        self.requests_post_patcher = patch('requests.post', side_effect=mock_post)
        self.mock_get = self.requests_patcher.start()
        self.mock_post = self.requests_post_patcher.start()
    
    def create_subscription(self, topic, msg_type, timeout=5.0):
        """Create a subscription to monitor a topic"""
        self.messages[topic] = []
        self.message_received[topic] = Event()
        
        def callback(msg, topic_name=topic):
            self.messages[topic_name].append(msg)
            self.message_received[topic_name].set()
        
        self.node.create_subscription(
            msg_type,
            topic,
            callback,
            10
        )
    
    def wait_for_message(self, topic, timeout=5.0):
        """Wait for a message on a specific topic"""
        if topic not in self.message_received:
            raise ValueError(f"No subscription for topic {topic}")
            
        self.message_received[topic].clear()
        
        start_time = time.time()
        while not self.message_received[topic].is_set() and time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return self.message_received[topic].is_set()
    
    def get_last_message(self, topic):
        """Get the last received message for a topic"""
        if topic not in self.messages or not self.messages[topic]:
            return None
        return self.messages[topic][-1]
    
    def verify_publishing_rate(self, topic, expected_rate, duration=2.0):
        """Verify that a topic publishes at the expected rate"""
        start_time = time.time()
        message_count = len(self.messages.get(topic, []))
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        final_count = len(self.messages.get(topic, []))
        actual_rate = (final_count - message_count) / duration
        
        # Allow 20% tolerance in rate
        assert abs(actual_rate - expected_rate) <= expected_rate * 0.2, \
            f"Expected rate {expected_rate}Hz, got {actual_rate}Hz"
    
    def tearDown(self):
        """Clean up after each test"""
        self.requests_patcher.stop()
        self.requests_post_patcher.stop()