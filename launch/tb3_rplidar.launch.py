#!/usr/bin/env python3
# TurtleBot3 Burger bringup with an RPLIDAR A1M8 instead of the stock LDS.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    tb3_param = os.path.join(
        bringup_dir, 'param', os.environ.get('TURTLEBOT3_MODEL', 'burger') + '.yaml')

    use_lidar = LaunchConfiguration('use_lidar')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the RPLIDAR driver on this machine'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'turtlebot3_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'namespace': ''}.items(),
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            condition=IfCondition(use_lidar),
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'inverted': False,
            }],
            output='screen'),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param, {'namespace': ''}],
            arguments=['-i', '/dev/ttyACM0'],
            respawn=True,
            respawn_delay=2.0,
            output='screen'),
    ])
