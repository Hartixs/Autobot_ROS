import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Environment ───────────────────────────────────────────────────────────
    gpu_env        = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    nvidia_lib     = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    set_render_eng = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre2')

    # ── Arguments ─────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='./src/autobot_ros/worlds/warehouse.sdf',
        description='Path to .sdf world file'
    )
    world_config = LaunchConfiguration('world')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Package Directories ────────────────────────────────────────────────────
    pkg_autobot = get_package_share_directory('autobot_ros')
    pkg_slam    = get_package_share_directory('slam_toolbox')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')

    # ── Robot State Publisher ──────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autobot, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Gazebo Sim (server only — -s flag, no GUI overhead) ───────────────────
    # real_time_update_rate=0 in world SDF = run as fast as CPU allows
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 2 --render-engine ogre2 ', world_config],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Spawn Robot ────────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'my_bot',
            '-z',     '0.5'
        ],
        output='screen'
    )

    # ── ROS–Gazebo Bridge ──────────────────────────────────────────────────────
    bridge_params = os.path.join(pkg_autobot, 'config', 'bridge_params.yaml')
    bridge_qos    = os.path.join(pkg_autobot, 'config', 'bridge_qos.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }, bridge_qos],
        output='screen'
    )

    # ── Scan Filter ────────────────────────────────────────────────────────────
    # Custom node — no laser_filters package needed.
    # Strips returns >= 9.0m from /scan → publishes /scan_filtered
    # 9.0m cutoff is aggressive: eliminates all max-range (12m) ghost returns
    # with a 3m safety buffer. Real obstacles in warehouse are all < 9m anyway.
    laser_filter = Node(
        package='autobot_ros',
        executable='scan_filter',
        name='scan_filter',
        output='screen',
        parameters=[{
            'upper_threshold': 9.0,    # aggressive — cut anything >= 9m
            'lower_threshold': 0.05,
        }]
    )

    # ── SLAM Toolbox ───────────────────────────────────────────────────────────
    slam_params = os.path.join(
        pkg_autobot, 'config', 'mapper_params_online_async.yaml'
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file':  slam_params,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ── Launch Description ─────────────────────────────────────────────────────
    return LaunchDescription([
        gpu_env,
        nvidia_lib,
        set_render_eng,

        world_arg,
        use_sim_time_arg,

        rsp,
        gazebo,
        spawn_entity,
        bridge,

        # Scan filter first — SLAM and Nav2 should only ever see clean data
        TimerAction(period=2.0, actions=[laser_filter]),

        # SLAM after bridge is up
        TimerAction(period=3.0, actions=[slam]),

        # RViz last
        TimerAction(period=5.0, actions=[rviz]),
    ])