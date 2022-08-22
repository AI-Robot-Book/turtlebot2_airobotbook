from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import Command


def generate_launch_description():
    turtlebot2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'turtlebot2.launch.py'
                ])
            ]),
        )

    slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
        )

    xacro_file = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'urdf',
            'turtlebot2.urdf.xacro'
        ])

    robot_description = {
            'robot_description': Command(
                ['xacro ', xacro_file]
            )
        }

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description],
        )

    rviz_config = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'rviz',
            'slam.rviz'
        ])

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )

    return LaunchDescription([
        turtlebot2,
        slam_toolbox,
        robot_state_publisher,
        rviz,
    ])
