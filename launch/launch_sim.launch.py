import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Define arguments FIRST so they can be used below
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World to load'
    )

    world_config = LaunchConfiguration('world')

    # --- FIX 1: Force NVIDIA GPU settings ---
    # set_gpu_offload = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre')

    package_name='autobot_ros' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- FIX 2: Now we can safely use 'world_config' ---
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={
                    'gz_args': ['-r --render-engine ogre ', world_config]
                }.items()
             )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.5'],
                        output='screen')

    return LaunchDescription([
        # set_gpu_offload,
        world_arg,
        set_render_engine,
        rsp,
        gazebo,
        spawn_entity,
    ])