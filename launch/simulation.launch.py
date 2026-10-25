from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world').perform(context)

    pkgSocNav = FindPackageShare('social_nav')
    pkgGz = FindPackageShare('ros_gz_sim')

    world_files = {
        'campus': 'campus_world.sdf',
        'airport': 'airport_terminal.sdf'
    }

    world_file_name = world_files.get(world_name, 'empty.sdf')

    world_path = PathJoinSubstitution([pkgSocNav, 'worlds', world_name, world_file_name]).perform(context)
    fuel_models_path = PathJoinSubstitution([pkgSocNav, 'worlds', world_name, 'fuel_models']).perform(context)
    models_path = PathJoinSubstitution([pkgSocNav, 'worlds', world_name, 'models']).perform(context)
    base_model_path = PathJoinSubstitution([pkgSocNav, 'models']).perform(context)

    resource_paths = f'{fuel_models_path}:{models_path}:{base_model_path}'

    try:
        rmf_assets = PathJoinSubstitution([FindPackageShare('rmf_demos_assets'), 'models']).perform(context)
        resource_paths += f':{rmf_assets}'
    except Exception:
        pass

    set_gz_res_path = AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_paths)

    plugin_dir = ':'.join([
        '/opt/ros/jazzy/lib/rmf_building_sim_gz_plugins',
        '/opt/ros/jazzy/lib/rmf_robot_sim_gz_plugins',
    ])
    
    set_sys_plugins = AppendEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_dir)
    set_gui_plugins = AppendEnvironmentVariable(name='GZ_GUI_PLUGIN_PATH', value=plugin_dir)

    gz_launch_path = PathJoinSubstitution([pkgGz, 'launch', 'gz_sim.launch.py']).perform(context)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    return [set_gz_res_path, set_sys_plugins, set_gui_plugins, gz_sim]

def generate_launch_description():
  return LaunchDescription([
      DeclareLaunchArgument(
          'world',
          default_value='empty',
          description='Yüklenecek harita adı (campus, airport vb.)',
      ),
      OpaqueFunction(function=launch_setup),
  ])
