#!/usr/bin/python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_dir = get_package_share_directory('ros2_cppserial')
    left_hydromast_params_file = os.path.join(params_dir, 'config', 'hydromast_n27.yaml')
    right_hydromast_params_file = os.path.join(params_dir, 'config', 'hydromast_n26.yaml')
    gps_params_file = os.path.join(params_dir, 'config', 'gps.yaml')
    yaml_files = [left_hydromast_params_file, right_hydromast_params_file, gps_params_file]

    for file in yaml_files:
        if os.path.exists(file):
            print("Found file: ", file)
        else:
            print("File not found: ", file)
            sys.exit(1)
    
    hydromast_left_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='hydromast_n27_interface',
        output='both',
        emulate_tty=True,
        parameters=[left_hydromast_params_file],
    )        
    hydromast_right_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='hydromast_n26_interface',
        output='both',
        emulate_tty=True,
        parameters=[right_hydromast_params_file],
    )    
    gps_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='gps_interface',
        output='both',
        emulate_tty=True,
        parameters=[gps_params_file],
    )

    return LaunchDescription([
        hydromast_left_interface_node,
        hydromast_right_interface_node,
        gps_interface_node,
    ])