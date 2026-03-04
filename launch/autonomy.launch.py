"""
autonomy_frontier.launch.py
─────────────────────────────────────────────────────────────────────────────
SECOND launch file — run this after compiled.launch.py is up.

compiled.launch.py  →  Gazebo + robot + bridge + SLAM + RViz
autonomy_frontier   →  Nav2 + Frontier Explorer (+ optional Coverage Planner)

DO NOT run SLAM here — compiled.launch.py already handles it.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package Directories ───────────────────────────────────────────────────
    pkg_autobot = get_package_share_directory('autobot_ros')
    pkg_nav2    = get_package_share_directory('nav2_bringup')

    # ── Arguments ────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    run_coverage_arg = DeclareLaunchArgument(
        'run_coverage',
        default_value='false',
        description='Launch coverage planner. Leave false during frontier exploration — only enable after map is fully built.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_coverage = LaunchConfiguration('run_coverage')

    # ── 1. Navigation2 ────────────────────────────────────────────────────────
    # SLAM is already running from compiled.launch.py — only Nav2 needed here.
    nav2_params = os.path.join(
        pkg_autobot, 'config', 'official_nav_params.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
        }.items()
    )

    # ── 2. Frontier Explorer Node ─────────────────────────────────────────────
    frontier_node = Node(
        package='r2d2_autonomy',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 3. Coverage Planner Node (disabled by default) ────────────────────────
    # Only launch manually after frontier exploration is complete:
    #   ros2 launch autobot_ros autonomy_frontier.launch.py run_coverage:=true
    coverage_node = Node(
        package='r2d2_autonomy',
        executable='coverage_planner',
        name='coverage_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(run_coverage)
    )

    # ── Launch Description ────────────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        run_coverage_arg,

        # 1. Nav2 — starts immediately (SLAM already up from compiled.launch.py)
        nav2_launch,

        # 2. Frontier Explorer — wait for Nav2 lifecycle to fully come up (~8s)
        TimerAction(period=8.0, actions=[frontier_node]),

        # 3. Coverage Planner — disabled by default, see run_coverage arg above
        TimerAction(period=8.0, actions=[coverage_node]),
    ])