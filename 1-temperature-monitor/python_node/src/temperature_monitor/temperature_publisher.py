#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import time
import signal
import sys
from datetime import datetime
import argparse

from temperature_interfaces.msg import TemperatureReading


class TemperaturePublisher(Node):
    """
    ROS2 Node that publishes simulated temperature readings with realistic variations.
    """
    
    def __init__(self):
        super().__init__('temperature_publisher')
        
        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('temp_range', 30.0)
        self.declare_parameter('sensor_id', 'sensor_001')
        self.declare_parameter('base_temp', 20.0)
        self.declare_parameter('location', 'Lab Room A')
        
        # Get parameter values
        raw_freq = self.get_parameter('frequency').get_parameter_value().double_value
        self.frequency = max(0.01, min(100.0, raw_freq))
        raw_range = self.get_parameter('temp_range').get_parameter_value().double_value
        self.temp_range = abs(raw_range) if raw_range > 0 else 5.0
        self.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        self.base_temp = self.get_parameter('base_temp').get_parameter_value().double_value
        self.location = self.get_parameter('location').get_parameter_value().string_value
        
        # Set up QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create publisher
        self.publisher = self.create_publisher(
            TemperatureReading, 
            'temperature_data', 
            qos_profile
        )
        
        # Create timer for publishing
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.publish_temperature)
        
        # Initialize variables for realistic temperature simulation
        self.start_time = time.time()
        self.message_count = 0
        
        # Logging
        self.get_logger().info(f'Temperature Publisher initialized:')
        self.get_logger().info(f'  - Sensor ID: {self.sensor_id}')
        self.get_logger().info(f'  - Frequency: {self.frequency} Hz')
        self.get_logger().info(f'  - Temperature range: ±{self.temp_range}°C')
        self.get_logger().info(f'  - Base temperature: {self.base_temp}°C')
        self.get_logger().info(f'  - Location: {self.location}')
        
        # Set up graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def generate_realistic_temperature(self):
        """
        Generate realistic temperature readings using sine wave + random noise.
        Simulates daily temperature variations with some random fluctuations.
        """
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Daily cycle (24 hours = 86400 seconds)
        # For demo purposes, use a shorter cycle (e.g., 5 minutes = 300 seconds)
        daily_cycle = 300.0  # 5 minutes for demo
        
        # Sine wave for daily temperature variation
        daily_variation = np.sin(2 * np.pi * elapsed_time / daily_cycle) * (self.temp_range * 0.3)
        
        # Add some shorter-term variations
        short_variation = np.sin(2 * np.pi * elapsed_time / 30.0) * (self.temp_range * 0.1)
        
        # Add random noise
        noise = np.random.normal(0, self.temp_range * 0.05)
        
        # Combine all variations
        temperature = self.base_temp + daily_variation + short_variation + noise
        
        return temperature
    
    def generate_humidity(self, temperature):
        """Generate realistic humidity based on temperature."""
        # Inverse relationship: higher temp -> lower humidity (simplified)
        base_humidity = 60.0
        temp_factor = (temperature - 20.0) * -0.5
        noise = np.random.normal(0, 5.0)
        
        humidity = base_humidity + temp_factor + noise
        return max(20.0, min(90.0, humidity))  # Clamp between 20-90%
    
    def publish_temperature(self):
        """Publish a temperature reading message."""
        try:
            # Generate temperature and humidity
            temperature = self.generate_realistic_temperature()
            humidity = self.generate_humidity(temperature)
            
            # Create message
            msg = TemperatureReading()
            msg.temperature = temperature
            msg.timestamp = self.get_clock().now().to_msg()
            msg.sensor_id = self.sensor_id
            msg.unit = 'Celsius'
            msg.humidity = humidity
            msg.location = self.location
            
            # Publish message
            self.publisher.publish(msg)
            self.message_count += 1
            
            # Log every 10th message to avoid spam
            if self.message_count % 10 == 0:
                self.get_logger().info(
                    f'Published reading #{self.message_count}: '
                    f'{temperature:.2f}°C, {humidity:.1f}% humidity'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing temperature: {str(e)}')
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info(f'Received signal {sig}, shutting down gracefully...')
        self.get_logger().info(f'Published {self.message_count} temperature readings')
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    """Main entry point for the temperature publisher node."""
    rclpy.init(args=args)
    
    try:
        temperature_publisher = TemperaturePublisher()
        
        # Spin the node
        rclpy.spin(temperature_publisher)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in temperature publisher: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()