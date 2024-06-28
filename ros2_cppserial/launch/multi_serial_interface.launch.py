import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    names = LaunchConfiguration('names').perform(context).split(',')
    print(f'Node names: {names}')

    def get_param_file(name):
        params_dir = get_package_share_directory('ros2_cppserial')
        return os.path.join(params_dir, 'config', name + '.yaml')

    # Create a Node action for each name
    node_actions = [
        Node(
            package='ros2_cppserial',
            executable='serial_interface',
            name=name + '_interface',
            output='screen',
            parameters=[get_param_file(name)]
        )
        for name in names
    ]

    return node_actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])