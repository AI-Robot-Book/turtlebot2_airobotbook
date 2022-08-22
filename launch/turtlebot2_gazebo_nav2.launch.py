from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    declare_map = DeclareLaunchArgument(
            'map',
            description='Full path to map file to load',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot2_airobotbook'),
                'map',
                'map.yaml'
            ]),
        )

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            description='Use simulation (Gazebo) clock if true',
            default_value='true',
        )

    declare_params_file = DeclareLaunchArgument(
            'params_file',
            description='Full path to param file to load',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot2_airobotbook'),
                'param',
                'airobotbook.yaml'
            ]),
        )

    config_map = LaunchConfiguration('map')

    config_use_sim_time = LaunchConfiguration('use_sim_time')

    config_params_file = LaunchConfiguration('params_file')

    turtlebot2_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'turtlebot2_gazebo.launch.py'
                ])
            ]),
        )

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'map': config_map,
                'use_sim_time': config_use_sim_time,
                'params_file': config_params_file}.items(),
        )

    rviz_config = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'rviz',
            'nav2_airobotbook_view.rviz'
        ])

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': config_use_sim_time}],
            output='screen',
        )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        declare_params_file,
        turtlebot2_gazebo,
        nav2,
        rviz,
    ])
