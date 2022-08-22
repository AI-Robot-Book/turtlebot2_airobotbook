from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    turtlebot2_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'turtlebot2_crane_gazebo.launch.py'
                ])
            ]),
        )

    rviz_config = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'rviz',
            'turtlebot2_crane.rviz'
        ])

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

    return LaunchDescription([
        turtlebot2_gazebo,
        rviz,
    ])
