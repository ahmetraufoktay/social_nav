from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_social_nav = FindPackageShare('social_nav')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    world_file = PathJoinSubstitution([
        pkg_social_nav, 'worlds', 'airport', 'airport_terminal.sdf'
    ])

    resource_paths = [
        PathJoinSubstitution([pkg_social_nav, 'worlds', 'airport', 'fuel_models']),
        ':',
        PathJoinSubstitution([pkg_social_nav, 'worlds', 'airport', 'models']),
        ':',
        PathJoinSubstitution([pkg_social_nav, 'models']),
    ]

    try:
        pkg_rmf_assets = FindPackageShare('rmf_demos_assets')
        resource_paths.extend([
            ':',
            PathJoinSubstitution([pkg_rmf_assets, 'models'])
        ])
    except Exception:
        pass

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_paths
    )

    plugin_dir = ':'.join([
        '/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins',
        '/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins',
    ])
    set_sys_plugins = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_dir
    )
    set_gui_plugins = AppendEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH', value=plugin_dir
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_sys_plugins,
        set_gui_plugins,
        gz_sim,
    ])
