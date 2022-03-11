#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='LD14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'topic_name': 'LiDAR/LD14'},
        {'port_name': '/dev/ttyUSB0'},
        {'frame_id': 'lidar_frame'}
      ]
    )
  ])
