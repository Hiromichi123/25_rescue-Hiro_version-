from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='core_2026',
            executable='mecanum_controller_node',
            name='mecanum_controller',
            output='screen',
            parameters=[{
                'wheel_radius_m': 0.05,
                'wheelbase_m': 0.18,
                'track_width_m': 0.18,
                'max_wheel_rpm': 300.0,
            }],
        ),
    ])
