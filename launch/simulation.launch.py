import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    EmitEvent,
    RegisterEventHandler
)
from launch.events import matches_action
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.actions import ExecuteProcess, TimerAction


def detection_python():
    """Python with torch/ultralytics (the package's .venv). Falls back to system python3."""
    path_file = os.path.join(get_package_share_directory('social_nav'), 'python_interpreter')
    try:
        with open(path_file) as f:
            python = f.read().strip()
    except OSError:
        return 'python3'
    return python if os.path.exists(python) else 'python3'


def generate_launch_description():
    soc_nav = FindPackageShare('social_nav')
    tb4_gz = FindPackageShare('turtlebot4_gz_bringup')
    ros_gz_sim = FindPackageShare('ros_gz_sim')
    nav2_bringup = FindPackageShare('nav2_bringup')

    dr_spaam_ros = FindPackageShare('dr_spaam_ros')

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
            'auto_undock',
            default_value='true',
            description='Açılışta robotu otomatik olarak dock\'tan çıkar'
        ),

        DeclareLaunchArgument(
            name='nav2',
            default_value='true',
            description='Whether to run Nav2'
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
            'mode',
            default_value='lifelong',
            choices=['amcl', 'mapping', 'lifelong'],
            description='Localisation mode: amcl | mapping | lifelong'
        ),

        DeclareLaunchArgument(
            'slam_params',
            default_value=PathJoinSubstitution([soc_nav, 'config', 'slam_lifelong.yaml']),
            description='Lifelong slam_toolbox parameter file'
        ),

        DeclareLaunchArgument(
            'human_detection',
            default_value='true',
            description='Enter true if you want human detection (DR-SPAAM + RGB-D) else false'
        ),

        DeclareLaunchArgument(
            'python',
            default_value=detection_python(),
            description='Python interpreter for YOLO and DR-SPAAM (defaults to the package .venv)'
        )
    ]

    ENV_VARIABLES = [
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=PathJoinSubstitution([soc_nav, 'worlds'])
        ),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=PathJoinSubstitution([soc_nav, 'models'])
        ),

        # Forcing X11 when Wayland fails
        SetEnvironmentVariable(
            name="QT_QPA_PLATFORM",
            value="xcb"

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
        arguments=['-d', PathJoinSubstitution([soc_nav, 'config', 'nav2_default.rviz'])],
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
                    'use_sim_time': LaunchConfiguration('use_sim_time'), 
                    'model': 'standard',
                    'x': '0.0',
                    'y': '0.0',
                    'z': '0.1',
                    'yaw': '0.0',
                    'nav2': 'false',
                    'localization': 'false',
                    'slam': 'false',
                    'rviz': 'false',
                }.items()
            )
        ],
        scoped=True
    )

    auto_undock = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/undock',
                 'irobot_create_msgs/action/Undock', '{}'],
            output='screen'
        )],
        condition=IfCondition(LaunchConfiguration('auto_undock'))
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('nav2'), "'.lower() == 'true' and '",
            LaunchConfiguration('mode'), "' != 'lifelong'"
        ])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': PythonExpression([
                "'True' if '", LaunchConfiguration('mode'), "' == 'mapping' else 'False'"
            ]),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'True',
        }.items()
    )

    lifelong_cond = IfCondition(PythonExpression([
        "'", LaunchConfiguration('nav2'), "' == 'true' and '",
        LaunchConfiguration('mode'), "' == 'lifelong'"
    ]))

    nav2_navigation_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    lifelong_slam = LifecycleNode(
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params'),
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'use_lifecycle_manager': False}
        ]
    )

    lifelong_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lifelong_slam),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    lifelong_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifelong_slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(lifelong_slam),
                transition_id=Transition.TRANSITION_ACTIVATE
            ))]
        )
    )

    lifelong_group = GroupAction(
        actions=[
            nav2_navigation_only,
            lifelong_slam,
            lifelong_activate,
            lifelong_configure,
        ],
        condition=lifelong_cond,
        scoped=False,
    )

    # Human Detection
    dr_spaam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([dr_spaam_ros, 'launch', 'dr_spaam_ros.launch.py']))
    )

    yolo_node = Node(
        package='social_nav',
        executable='yolo_node',
        parameters=[{'publish_debug_image': True, 'conf_threshold': 0.4}]
    )

    human_detector = Node(
        package="social_nav",
        executable='human_detector',
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
    )

    torch_nodes = GroupAction(
        actions=[
            SetLaunchConfiguration('launch-prefix', LaunchConfiguration('python')),
            dr_spaam,
            yolo_node,
        ],
        scoped=True
    )

    human_detection_group = GroupAction(
        actions=[torch_nodes, human_detector],
        condition=IfCondition(LaunchConfiguration('human_detection'))
    )

    return LaunchDescription(
        ENV_VARIABLES + ARGS 
        + [
            clock_bridge,
            cmd_vel_relay,
            rviz,
            gz_sim,
            tb4_spawn,
            auto_undock,
            nav2_stack,
            lifelong_group,
            human_detection_group,
    ])
