#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import unittest
from unittest.mock import patch, MagicMock, Mock
from rclpy.parameter import Parameter
import numpy as np
from builtin_interfaces.msg import Time

from temperature_monitor.temperature_publisher import TemperaturePublisher
from temperature_interfaces.msg import TemperatureReading


class TestTemperaturePublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.mock_publisher = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def test_parameter_initialization(self):
        """Test that parameters are properly initialized with default values"""
        with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher):
            with patch.object(TemperaturePublisher, 'create_timer'):
                publisher = TemperaturePublisher()
                
                self.assertEqual(publisher.frequency, 1.0)
                self.assertEqual(publisher.temp_range, 30.0)
                self.assertEqual(publisher.sensor_id, 'sensor_001')
                self.assertEqual(publisher.base_temp, 20.0)
                self.assertEqual(publisher.location, 'Lab Room A')

    def test_temperature_generation(self):
        """Test that temperature generation produces realistic values"""
        with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher):
            with patch.object(TemperaturePublisher, 'create_timer'):
                publisher = TemperaturePublisher()
                publisher.temp_range = 5.0
                publisher.base_temp = 25.0
                
                # Test multiple generations to ensure consistency
                temperatures = [publisher.generate_realistic_temperature() for _ in range(100)]
                
                # Check that temperatures are within expected range
                for temp in temperatures:
                    self.assertGreaterEqual(temp, 15.0)  # 25 - 10 (buffer)
                    self.assertLessEqual(temp, 35.0)     # 25 + 10 (buffer)

    def test_humidity_generation(self):
        """Test that humidity generation produces valid values"""
        with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher):
            with patch.object(TemperaturePublisher, 'create_timer'):
                publisher = TemperaturePublisher()
                
                # Test with various temperatures
                test_temps = [10.0, 20.0, 30.0, 40.0]
                for temp in test_temps:
                    humidity = publisher.generate_humidity(temp)
                    self.assertGreaterEqual(humidity, 20.0)
                    self.assertLessEqual(humidity, 90.0)

    def test_message_publishing(self):
        """Test that messages are properly formatted and published"""
        # Create a dedicated mock for the temperature publisher only
        mock_temperature_publisher = MagicMock()
        
        def create_publisher_side_effect(msg_type, topic, qos_profile):
            if topic == 'temperature_data':
                return mock_temperature_publisher
            # Return a different mock for other topics (like parameter events)
            return MagicMock()
        
        with patch.object(TemperaturePublisher, 'create_publisher', side_effect=create_publisher_side_effect):
            with patch.object(TemperaturePublisher, 'create_timer'):
                with patch('time.time', return_value=1234567890):
                    publisher = TemperaturePublisher()
                    
                    # Create a proper Time message instead of a MagicMock
                    mock_time = Time(sec=1234567890, nanosec=0)
                    publisher.get_clock = MagicMock()
                    publisher.get_clock().now().to_msg.return_value = mock_time
                    
                    publisher.publish_temperature()
                    
                    # Verify the temperature publisher was called once
                    mock_temperature_publisher.publish.assert_called_once()
                    
                    # Verify message content
                    published_msg = mock_temperature_publisher.publish.call_args[0][0]
                    self.assertIsInstance(published_msg, TemperatureReading)
                    self.assertEqual(published_msg.sensor_id, 'sensor_001')
                    self.assertEqual(published_msg.unit, 'Celsius')
                    self.assertEqual(published_msg.location, 'Lab Room A')
                    self.assertEqual(published_msg.timestamp.sec, mock_time.sec)
                    self.assertEqual(published_msg.timestamp.nanosec, mock_time.nanosec)

    def test_extreme_frequency_values(self):
        """Test publisher with very high/low frequencies"""
        # Test low frequency (0.1 Hz → 10.0 sec period)
        with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher), \
            patch.object(TemperaturePublisher, 'create_timer') as mock_create_timer, \
            patch.object(TemperaturePublisher, 'get_parameter') as mock_get_param:
            
            # Mock the parameter retrieval for frequency
            mock_frequency_param = MagicMock()
            mock_frequency_param.get_parameter_value.return_value.double_value = 0.1
            mock_get_param.return_value = mock_frequency_param
            
            publisher = TemperaturePublisher()
            
            expected_period = 1.0 / 0.1  # 10.0 seconds
            mock_create_timer.assert_called_once()
            actual_period = mock_create_timer.call_args[0][0]
            self.assertAlmostEqual(actual_period, expected_period, delta=0.01)
            
            # Clean up
            publisher.destroy_node()
        
        # Test high frequency (10.0 Hz → 0.1 sec period)
        with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher), \
            patch.object(TemperaturePublisher, 'create_timer') as mock_create_timer, \
            patch.object(TemperaturePublisher, 'get_parameter') as mock_get_param:
            
            # Mock the parameter retrieval for frequency
            mock_frequency_param = MagicMock()
            mock_frequency_param.get_parameter_value.return_value.double_value = 10.0
            mock_get_param.return_value = mock_frequency_param
            
            publisher = TemperaturePublisher()
            
            expected_period = 1.0 / 10.0  # 0.1 seconds
            mock_create_timer.assert_called_once()
            actual_period = mock_create_timer.call_args[0][0]
            self.assertAlmostEqual(actual_period, expected_period, delta=0.001)
            
            # Clean up
            publisher.destroy_node()

    def test_invalid_temperature_ranges(self):
        """Ensure invalid temp ranges are handled gracefully (e.g., corrected or logged)"""
        with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher), \
            patch.object(TemperaturePublisher, 'create_timer'):

            # Set an invalid negative temp_range
            self.node.set_parameters([Parameter('temp_range', Parameter.Type.DOUBLE, -5.0)])

            # Create the publisher — should handle negative range gracefully
            publisher = TemperaturePublisher()

            # It should use abs() or clamp to positive
            self.assertGreater(publisher.temp_range, 0.0)  # Should auto-correct

    def test_timestamp_consistency(self):
        """Verify timestamp is properly set from ROS clock"""
        with patch.object(TemperaturePublisher, 'get_clock') as mock_get_clock:
            # Create a proper mock clock that returns a valid Time message
            mock_clock = Mock()
            mock_time_msg = Time()
            mock_time_msg.sec = 123456
            mock_time_msg.nanosec = 789000000
            mock_clock.now.return_value.to_msg.return_value = mock_time_msg
            mock_get_clock.return_value = mock_clock

            with patch.object(TemperaturePublisher, 'create_publisher', return_value=self.mock_publisher), \
                patch.object(TemperaturePublisher, 'create_timer'):
                publisher = TemperaturePublisher()

            with patch.object(publisher.publisher, 'publish') as mock_publish:
                publisher.publish_temperature()

            # Now call_args should not be None
            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertIsInstance(published_msg, TemperatureReading)
            self.assertEqual(published_msg.timestamp.sec, 123456)
            self.assertEqual(published_msg.timestamp.nanosec, 789000000)

if __name__ == '__main__':
    unittest.main()