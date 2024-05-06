import launch


def generate_launch_description():
    gps_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/gps_data',
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        gps_recording
        ])
