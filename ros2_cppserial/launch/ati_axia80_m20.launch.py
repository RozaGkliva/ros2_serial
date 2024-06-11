#!/usr/bin/python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_dir = get_package_share_directory('ros2_cppserial')
    ati_params_file = os.path.join(params_dir, 'config', 'ati_axia80_m20.yaml')
    yaml_files = [ati_params_file]

    for file in yaml_files:
        if os.path.exists(file):
            print("Found file: ", file)
        else:
            print("File not found: ", file)
            sys.exit(1)
    
    ati_interface_node = Node(
        package='ros2_cppserial',
        executable='serial_interface',
        name='ati_ft_interface',
        output='both',
        emulate_tty=True,
        parameters=[ati_params_file],
    )


    return LaunchDescription([
        ati_interface_node,
    ])