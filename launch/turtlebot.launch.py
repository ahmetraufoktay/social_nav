from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    soc_nav = FindPackageShare('social_nav')
    tb4_gz = FindPackageShare('turtlebot4_gz_bringup')

    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([soc_nav, 'worlds'])
    )

    world_arg = DeclareLaunchArgument(
        name='world', 
        default_value='depot', 
        description="Gazebo world name"
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([soc_nav, 'maps', 'depot_map.yaml']),
        description='Map Path'
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Nav2'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='SLAM'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='true',
        description='Localization'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='RViz'
    )

    tb4_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tb4_gz, 'launch', 'turtlebot4_gz.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'nav2': LaunchConfiguration('nav2'),
            'slam': LaunchConfiguration('slam'),
            'localization': LaunchConfiguration('localization'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )
    
    return LaunchDescription([
        gz_resource_path,
        world_arg,
        map_arg,
        nav2_arg,
        slam_arg,
        localization_arg,
        rviz_arg,
        tb4_gz_launch
    ])
