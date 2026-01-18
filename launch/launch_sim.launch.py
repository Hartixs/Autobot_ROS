import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gpu_env = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    nvidia_lib = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    # 1. Define arguments FIRST so they can be used below
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World to load'
    )

    world_config = LaunchConfiguration('world')

    # --- FIX 1: Force NVIDIA GPU settings ---
    # set_gpu_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre2')

    package_name='autobot_ros' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- FIX 2: Now we can safely use 'world_config' ---
# Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            '-r -v 4 ', 
            '--render-engine ogre2 ',
            world_config
        ], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.5'],
                        output='screen')

    # --- Bridge Node Configuration ---
    # saved YAML file to src/autobot_ros/config/bridge_params.yaml
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'bridge_params.yaml'
    )
    
    bridge_qos = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'bridge_qos.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
        },bridge_qos],
        output='screen'
    )

    return LaunchDescription([
        # set_gpu_offload,
        world_arg,
        set_render_engine,
        rsp,
        gpu_env,
        nvidia_lib,
        gazebo,
        spawn_entity,
        start_gazebo_ros_bridge_cmd, # Added the bridge here
    ])