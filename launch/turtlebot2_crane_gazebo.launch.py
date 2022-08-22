from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    world_file = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'worlds',
            'airobotbook.world'
        ])

    declare_arg_gui = DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless.',
        )

    declare_arg_server = DeclareLaunchArgument(
            'server',
            default_value='true',
            description='Set to "false" not to run gzserver.',
        )

    set_env = SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=PathJoinSubstitution([
                    FindPackageShare('turtlebot2_airobotbook'),
                    'models'
            ]),
        )

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch', 'gzserver.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
        )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch', 'gzclient.launch.py'
                ])
            ]),
            condition=IfCondition(LaunchConfiguration('gui')),
        )

    xacro_file = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'urdf',
            'turtlebot2_crane.urdf.xacro'
        ])

    robot_description = {
            'robot_description': Command(
                ['xacro ', xacro_file, ' use_gazebo:=', 'true']
            )
        }

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description],
        )

    spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot2_crane',
                '-x', '0.0', '-y', '0.0', '-z', '0.01',
                '-topic', '/robot_description'],
            output='screen',
        )

    steering = ExecuteProcess(
            cmd=['rqt_robot_steering'],
            shell=True,
            output='screen'
        )

    jsc_pram_file = PathJoinSubstitution([
            FindPackageShare('turtlebot2_airobotbook'),
            'param',
            'joint_state_controller.yaml'
        ])

    spawn_joint_state_controller = ExecuteProcess(
            cmd=[
                'ros2 run controller_manager spawner.py joint_state_controller'
                ' -p', jsc_pram_file
            ],
            shell=True,
            output='screen',
        )

    spawn_arm_controller = ExecuteProcess(
            cmd=['ros2 run controller_manager spawner.py crane_plus_arm_controller'],
            shell=True,
            output='screen',
        )

    spawn_gripper_controller = ExecuteProcess(
            cmd=['ros2 run controller_manager spawner.py crane_plus_gripper_controller'],
            shell=True,
            output='screen',
        )

    return LaunchDescription([
        set_env,
        declare_arg_gui,
        declare_arg_server,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        steering,
        spawn_joint_state_controller,
        spawn_arm_controller,
        spawn_gripper_controller,
    ])
