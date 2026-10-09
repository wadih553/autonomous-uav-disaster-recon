#!/usr/bin/env python3
"""
uav_bringup.launch.py
------------------------
Single launch file that brings up the entire onboard (Raspberry Pi) stack,
matching the boot sequence described in the FYP report (Ch. 5.2, "Flight
check steps"):

  1. MAVROS <-> Pixhawk bridge (MAVLink over UART @ 57600 baud, Telemetry port)
  2. YDLiDAR X4 Pro driver (360-degree scan -> obstacle_avoidance)
  3. drone_pkg nodes: camera, environmental sensors, mission receiver
  4. navigator_pkg: mission execution
  5. obstacle_avoidance: real-time LiDAR-based avoidance
  6. rosbridge_server: WebSocket bridge to the Flask ground station
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    lidar_port = LaunchConfiguration('lidar_port')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/ttyAMA0:57600',
            description='Serial port + baud rate to the Pixhawk 2.4.8 (Telemetry 3 / UART)'
        ),
        DeclareLaunchArgument(
            'gcs_url', default_value='udp://@',
            description='Optional secondary GCS passthrough (e.g. QGroundControl over Wi-Fi)'
        ),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='USB-to-UART adapter port for the YDLiDAR X4 Pro'
        ),
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='WebSocket port exposed to the Flask ground station'
        ),

        # 1. MAVROS <-> Pixhawk
        Node(
            package='mavros', executable='mavros_node', name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
        ),

        # 2. YDLiDAR X4 Pro driver
        Node(
            package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
            name='ydlidar_node', output='screen',
            parameters=[{
                'port': lidar_port,
                'frame_id': 'laser_frame',
                'baudrate': 128000,
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 5,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'range_min': 0.1,
                'range_max': 12.0,
                'frequency': 8.0,
            }],
        ),

        # 3. drone_pkg
        Node(package='drone_pkg', executable='camera_node', name='camera_node', output='screen'),
        Node(package='drone_pkg', executable='env_sensor_node', name='env_sensor_node', output='screen'),
        Node(package='drone_pkg', executable='mission_receiver_node', name='mission_receiver_node', output='screen'),

        # 4. navigator_pkg
        Node(package='navigator_pkg', executable='navigator_node', name='navigator_node', output='screen'),

        # 5. obstacle_avoidance
        Node(package='obstacle_avoidance', executable='obstacle_avoidance_node',
             name='obstacle_avoidance_node', output='screen'),

        # 6. ROSBridge WebSocket server (Pi <-> Flask ground station)
        Node(
            package='rosbridge_server', executable='rosbridge_websocket',
            name='rosbridge_websocket', output='screen',
            parameters=[{'port': rosbridge_port}],
        ),
    ])
