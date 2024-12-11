from launch import LaunchDescription
from launch.actions import LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 livox_ros_driver2 的 msg_MID360_launch.py 路径
    livox_launch_path = PathJoinSubstitution([
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2',
        'msg_MID360_launch.py'
    ])

    # 获取 fast_lio 的 mapping.launch.py 路径
    fast_lio_launch_path = PathJoinSubstitution([
        get_package_share_directory('fast_lio'),
        'launch',
        'mapping.launch.py'
    ])

    return LaunchDescription([
        # 启动 livox_ros_driver2
        LogInfo(msg="Starting livox_ros_driver2 msg_MID360_launch.py"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch_path)
        ),

        # 启动 fast_lio 并传递参数
        LogInfo(msg="Starting fast_lio mapping.launch.py rviz:=false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch_path),
            launch_arguments={'rviz': 'false'}.items()
        ),

        # 启动 ros_tools 的 lidar_data_node
        LogInfo(msg="Starting ros_tools lidar_data_node"),
        Node(
            package='ros_tools',
            executable='lidar_data_node',
            output='screen',
            name='lidar_data_node'
        ),

        # 启动 vel_control 的 position_publisher
        LogInfo(msg="Starting vel_control position_publisher"),
        Node(
            package='vel_control',
            executable='position_publisher',
            output='screen',
            name='position_publisher'
        ),

        # 启动 elrs_receiver 的 elrs_receiver_node
        LogInfo(msg="Starting elrs_receiver elrs_receiver_node"),
        Node(
            package='elrs_receiver',
            executable='elrs_receiver_node',
            output='screen',
            name='elrs_receiver_node'
        ),

        # 启动 ros_tools 的 ground_camera_node
        LogInfo(msg="Starting ros_tools ground_camera_node"),
        Node(
            package='ros_tools',
            executable='ground_camera_node',
            output='screen',
            name='ground_camera_node'
        ),

        # 启动 vision 的 ball_calibration.py
        LogInfo(msg="Starting vision ball_calibration.py"),
        Node(
            package='vision',
            executable='ball_calibration.py',
            output='screen',
            name='ball_calibration'
        ),

        # 启动 ros_tools 的 front_camera_node
        LogInfo(msg="Starting ros_tools front_camera_node"),
        Node(
            package='ros_tools',
            executable='front_camera_node',
            output='screen',
            name='front_camera_node'
        ),
    ])
