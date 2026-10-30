from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    soc_nav = FindPackageShare('social_nav')
    tb4_gz = FindPackageShare('turtlebot4_gz_bringup')
    ros_gz_sim = FindPackageShare('ros_gz_sim')
    nav2_bringup = FindPackageShare('nav2_bringup')

    ARGS = [
        DeclareLaunchArgument(
            name='world',
            default_value='depot',
            description='Gazebo world name'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=PathJoinSubstitution([soc_nav, 'maps', 'depot_map.yaml']),
            description='Map YAML file path'
        ),

        DeclareLaunchArgument(
            name='params_file',
            default_value=PathJoinSubstitution([soc_nav, 'config', 'nav2_params.yaml']),
            description='Nav2 parameters file'
        ),

        DeclareLaunchArgument(
            name='nav2',
            default_value='true',
            description='Whether to run Nav2'
        ),

        DeclareLaunchArgument(
            name='slam',
            default_value='false',
            description='Whether to start SLAM'
        ),

        DeclareLaunchArgument(
            name='localization',
            default_value='false',
            description='Whether to start Localization'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Whether to start RViz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
    ]

    ENV_VARIABLES = [
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=PathJoinSubstitution([soc_nav, 'worlds'])
        ),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=PathJoinSubstitution([soc_nav, 'models'])
        )
    ]

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', "/actors/poses@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"]
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel_nav', '/diffdrive_controller/cmd_vel'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([nav2_bringup, 'rviz', 'nav2_default_view.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 --render-engine ogre2 ', LaunchConfiguration('world'), '.sdf']
        }.items()
    )


    tb4_spawn = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([tb4_gz, 'launch', 'turtlebot4_spawn.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'), # Not: use_sim_time_arg yerine use_sim_time olmalı
                    'model': 'standard',
                    'x': '0.0',
                    'y': '0.0',
                    'z': '0.1',
                    'yaw': '0.0',
                    'nav2': 'false',
                    'localization': LaunchConfiguration('localization'),
                    'slam': LaunchConfiguration('slam'),
                    'rviz': 'false',
                }.items()
            )
        ],
        scoped=True
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('nav2')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'False',
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    return LaunchDescription(
        ENV_VARIABLES + ARGS 
        + [
            gz_sim,
            clock_bridge,
            tb4_spawn,
            cmd_vel_relay,
            nav2_stack,
            rviz
    ])
