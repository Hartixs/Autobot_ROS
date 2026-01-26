import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. RTAB-Map in LOCALIZATION Mode
    # We remove '--delete_db_on_start' so it loads your saved map.
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'database_path': '~/.ros/rtabmap.db',
            # --- INPUTS ---
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'scan_topic': '/scan',
            'odom_topic': '/odom',
            
            # --- SYNC SETTINGS ---
            'qos': '1',
            'approx_sync': 'true',
            'queue_size': '50', 

            # --- LOCALIZATION SETTINGS ---
            # Mem/IncrementalMemory false: Stops adding new data to the map (Localization only)
            # Mem/InitWMWithAllNodes true: Forces the whole map to load so Nav2 sees it all
            'rtabmap_args': '--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true --Grid/Sensor 0 --RGBD/ProximityPathMaxNeighbors 10',
            
            # --- COMMON CONFIG ---
            'visual_odometry': 'false',
            'subscribe_odom_info': 'false',
            'subscribe_scan': 'true',
            'frame_id': 'base_link',
            'rtabmap_viz': 'true', # Keep this to see the 3D map
            'args': '--Reg/Strategy 1 --Reg/Force3DoF true --RGBD/NeighborLinkRefining true' 
        }.items()
    )

    # 2. Nav2 Bringup (Navigation Stack Only)
    # We use 'navigation_launch.py' because we don't need 'map_server' or 'amcl' (RTAB-Map does that).
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            
            'params_file': os.path.join(get_package_share_directory('autobot_ros'), 'config', 'nav_params.yaml') 
        }.items()
    )
    return LaunchDescription([
        rtabmap_launch,
        nav2_launch
    ])