#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_dir = get_package_share_directory('ros2_cppserial')
    hydromast_params_file = os.path.join(params_dir, 'config', 'serial.yaml')
    gps_params_file = os.path.join(params_dir, 'config', 'gps.yaml')

    if os.path.exists(hydromast_params_file):
        print("Found file: ", hydromast_params_file)
    else:
        print("File not found: ", hydromast_params_file)
    
    hydromast_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='hydromast_interface',
        output='both',
        emulate_tty=True,
        # parameters=[{"port": "/dev/ttyACM0"}],
        parameters=[hydromast_params_file],
    )    
    gps_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='gps_interface',
        output='both',
        emulate_tty=True,
        # parameters=[{"port": "/dev/ttyACM0"}],
        parameters=[gps_params_file],
    )

    return LaunchDescription([
        hydromast_interface_node,
        # gps_interface_node,
    ])