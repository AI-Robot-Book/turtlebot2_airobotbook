from math import radians
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    kobuki = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('kobuki_node'),
                    'launch', 'kobuki_node-launch.py'
                ])
            ]),
        )

    steering = ExecuteProcess(
            cmd=['rqt_robot_steering'],
            shell=True,
            output='screen'
        )

    urg = Node(
            package='urg_node',
            executable='urg_node_driver',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyACM_urg'}],
        )

    x = 0.105  # [m] 前方向が正
    y = 0.000  # [m] 左方向が正
    z = 0.190  # [m] 上方向が正
    yaw_deg = 0    # [deg]
    pitch_deg = 0  # [deg]
    roll_deg = 0   # [deg]
    sx = str(x)
    sy = str(y)
    sz = str(z)
    syaw = str(radians(yaw_deg))
    spitch = str(radians(pitch_deg))
    sroll = str(radians(roll_deg))
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[sx, sy, sz, syaw, spitch, sroll,
                   'base_footprint', 'laser'],
    )

    return LaunchDescription([
        kobuki,
        steering,
        urg,
        static_transform_publisher,
    ])
