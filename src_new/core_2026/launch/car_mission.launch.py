import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    real_robot_odom_topic = LaunchConfiguration('real_robot_odom_topic')

    launch_args = [
        DeclareLaunchArgument('real_robot_odom_topic', default_value='/aft_mapped_to_init'),
    ]

    lidar_data_node = Node(
        package='ros2_tools',
        executable='lidar_data_node',
        parameters=[{
            'use_simulation': False,
            'simulation_odom_topic': '/absolute_pose',
            'real_robot_odom_topic': real_robot_odom_topic,
        }],
        output='screen',
    )

    car_mission_node = Node(
        package='core_2026',
        executable='car_mission_node',
        name='car_mission_executor',
        output='screen',
    )

    mecanum_controller_node = Node(
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
    )

    return launch.LaunchDescription(
        launch_args + [
            lidar_data_node,
            car_mission_node,
            mecanum_controller_node,
        ]
    )
