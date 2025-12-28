import launch
import launch_ros.actions


def generate_launch_description():
    lim_node = launch_ros.actions.Node(
        package='speed_lim_pkg',
        executable='lim_node',
        output='screen',
        parameters=[
            {'max_linear': 0.5},
            {'max_angular': 1.0}
        ]
    )

    return launch.LaunchDescription([lim_node])
