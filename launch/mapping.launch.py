import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            
            # --- INPUT TOPICS ---
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'scan_topic': '/scan',
            'odom_topic': '/odom',
            
            # --- SYNC & BUFFER SETTINGS (The Fix) ---
            'qos': '1',                # Reliable (Matches your Bridge)
            'approx_sync': 'true',     # Required for Sim
            'queue_size': '50',        # INCREASED: Buffers more 720p frames to find a match
            
            # --- MAPPING CONFIG ---
            'visual_odometry': 'false',
            'subscribe_odom_info': 'false',
            'subscribe_scan': 'true',
            'frame_id': 'base_link',
            'rtabmap_viz': 'true',
            
            # --- ARGUMENTS (Silencing Warnings + Drift Fix) ---
            # We pass the warning suppressors here instead of as parameters
            'rtabmap_args': '--delete_db_on_start --Grid/Sensor 0 --RGBD/ProximityPathMaxNeighbors 10',
            
            # --- DRIFT FIXES ---
            'args': '--Reg/Strategy 1 --Reg/Force3DoF true --RGBD/NeighborLinkRefining true' 
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])