import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to your mask yaml
    pkg_share = get_package_share_directory('autobot_ros')
    mask_yaml_file = '/home/arvind/swarm_ws/keepout.yaml' 

    return LaunchDescription([
        # 2. Map Server for the Mask
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False, # Change to True if using Gazebo
                'yaml_filename': mask_yaml_file
            }],
            # --- CRITICAL FIX START ---
            # This forces the node to publish on /keepout_mask instead of /map
            remappings=[
                ('map', 'keepout_mask'),
                ('map_server/load_map', 'keepout_mask/load_map'),
                ('map_server/transition_event', 'keepout_mask/transition_event'),
            ]
            # --- CRITICAL FIX END ---
            ),

        # 3. Costmap Filter Info Server (Tells Nav2 how to use the mask)
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False, # Change to True if using Gazebo
                'type': 0,
                'filter_info_topic': 'costmap_filter_info',
                'mask_topic': 'keepout_mask', # Must match the remapping above!
                'base_frame_id': 'map',
                'multiplier': 1.0
            }]),

        # 4. Lifecycle Manager (Manages ONLY these filter nodes)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_filters', # Unique name prevents conflict with main manager
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False, # Change to True if using Gazebo
                'autostart': True,
                'node_names': ['filter_mask_server', 'costmap_filter_info_server']
            }])
    ])