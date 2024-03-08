#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_dir = get_package_share_directory('ros2_pyserial')
    params_file = os.path.join(params_dir, 'config', 'serial.yaml')
    
    serial_interface_node = Node(
        package='ros2_pyserial',
        executable='serial_interface',
        name='serial_interface',
        output='both',
        parameters=[params_file],
    )

    return LaunchDescription([
        serial_interface_node
    ])

    