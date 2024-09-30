import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    names = LaunchConfiguration('names').perform(context).split(',')
    print(f'Node names: {names}')

    declared_package_name = DeclareLaunchArgument(
        'package_name',
        default_value='ros2_cppserial',
        description='Package name to search for the config file')
    package_name = LaunchConfiguration('package_name')

    def get_param_file(name):
        # params_dir = get_package_share_directory('ros2_cppserial')
        # return os.path.join(params_dir, 'config', name + '.yaml')
    
        config_file_path = PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            name +
            '.yaml'  # Replace with your actual config file name
        ])
        return config_file_path

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