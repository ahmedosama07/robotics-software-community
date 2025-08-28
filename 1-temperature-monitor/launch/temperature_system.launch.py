#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the temperature monitoring system.
    This demonstrates how you could launch the system if running natively.
    """
    
    # Declare launch arguments
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='1.0',
        description='Publishing frequency in Hz'
    )
    
    temp_range_arg = DeclareLaunchArgument(
        'temp_range',
        default_value='30.0',
        description='Temperature variation range in Celsius'
    )
    
    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id',
        default_value='sensor_001',
        description='Unique sensor identifier'
    )
    
    base_temp_arg = DeclareLaunchArgument(
        'base_temp',
        default_value='20.0',
        description='Base temperature in Celsius'
    )
    
    # Temperature publisher node
    publisher_node = Node(
        package='temperature_monitor',
        executable='temperature_publisher',
        name='temperature_publisher',
        parameters=[{
            'frequency': LaunchConfiguration('frequency'),
            'temp_range': LaunchConfiguration('temp_range'),
            'sensor_id': LaunchConfiguration('sensor_id'),
            'base_temp': LaunchConfiguration('base_temp'),
        }],
        output='screen'
    )
    
    # Temperature subscriber node
    subscriber_node = Node(
        package='temperature_monitor',
        executable='temperature_subscriber',
        name='temperature_subscriber',
        output='screen'
    )
    
    # Docker compose execution (alternative to native ROS2)
    docker_compose_up = ExecuteProcess(
        cmd=['docker compose', 'up', '--build'],
        name='docker_compose',
        output='screen',
        cwd='.'
    )
    
    return LaunchDescription([
        frequency_arg,
        temp_range_arg,
        sensor_id_arg,
        base_temp_arg,
        
        # Uncomment these for native ROS2 launch:
        # publisher_node,
        # subscriber_node,
        
        # For Docker deployment:
        docker_compose_up,
    ])