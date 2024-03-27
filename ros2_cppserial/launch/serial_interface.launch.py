#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_dir = get_package_share_directory('ros2_cppserial')
    params_file = os.path.join(params_dir, 'config', 'hydromast.yaml')

    if os.path.exists(params_file):
        print("Found file: ", params_file)
    else:
        print("File not found: ", params_file)
    
    serial_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='serial_interface',
        output='both',
        emulate_tty=True,
        # parameters=[{"port": "/dev/ttyACM0"}],
        parameters=[params_file],
    )

    return LaunchDescription([
        serial_interface_node
    ])