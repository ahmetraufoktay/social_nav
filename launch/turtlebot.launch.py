from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- Argümanlar ----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='airport',
        description='Yüklenecek harita adı (campus, airport vb.)',
    )
    # Gazebo SDF dosyasının içindeki <world name="..."> değeri.
    # Spawn servisi (/world/<isim>/create) bu isimle çağrılır.
    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='default',
        description='SDF içindeki <world name="..."> değeri',
    )
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='RViz başlat')
    x_arg = DeclareLaunchArgument('x', default_value='11.29', description='Robot X koordinatı')
    y_arg = DeclareLaunchArgument('y', default_value='-45.88', description='Robot Y koordinatı')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Robot Z koordinatı')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Robot Yaw açısı')
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay', default_value='5.0', description='Gazebo açıldıktan sonra spawn gecikmesi (sn)'
    )

    # ---- TurtleBot4 modelleri/mesh'leri için Gazebo kaynak yolu ----
    # Jazzy'de 'turtlebot4_gz_description' paketi yok; turtlebot4_simulator'ın
    # kendi sim.launch.py'ındaki paketlerle aynı yolları ekliyoruz.
    paths = []
    for pkg in (
        'turtlebot4_description',
        'irobot_create_description',
        'irobot_create_gz_bringup',
    ):
        paths.append(str(Path(get_package_share_directory(pkg)).parent.resolve()))
    paths.append(str(Path(get_package_share_directory('turtlebot4_gz_bringup')) / 'worlds'))

    tb4_resource_paths = ':'.join(paths)
    set_tb4_res_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', tb4_resource_paths)

    # ---- Mevcut dünya (harita) launch dosyası ----
    pkg_soc_nav = FindPackageShare('social_nav')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_soc_nav, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # ---- TurtleBot4 spawn (robot_description, köprüler, create servisi) ----
    pkg_tb4_gz_bringup = FindPackageShare('turtlebot4_gz_bringup')
    tb4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_gz_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('gz_world_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    # Gazebo'nun tamamen açılmasını bekle
    delayed_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[tb4_spawn],
    )

    return LaunchDescription([
        world_arg,
        gz_world_name_arg,
        namespace_arg,
        rviz_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_delay_arg,
        set_tb4_res_path,
        world_launch,
        delayed_spawn,
    ])
