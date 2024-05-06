import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', 
            '/gps_data',
            '/hydromast_n26_data',
            '/hydromast_n27_data',
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
