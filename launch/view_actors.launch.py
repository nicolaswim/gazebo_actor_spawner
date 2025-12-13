import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. CONFIGURATION ---
    user_models_path = "/home/wim/Documents/gazebo_actor_spawner/models"
    system_models_path = "/usr/share/gazebo-11/models" # Standard Ubuntu Path
    
    # --- 2. ENVIRONMENT SETUP ---
    gz_env = os.environ.copy()
    gz_env['QT_QPA_PLATFORM'] = 'xcb'
    if 'LD_PRELOAD' in gz_env: del gz_env['LD_PRELOAD']
    if 'WAYLAND_DISPLAY' in gz_env: del gz_env['WAYLAND_DISPLAY']
    
    # FIX: Include System Paths so 'sun' and 'ground_plane' work
    if 'GAZEBO_MODEL_PATH' in gz_env:
        gz_env['GAZEBO_MODEL_PATH'] += os.pathsep + user_models_path
    else:
        # If we set this variable manually, we override default internal paths.
        # So we must manually re-add the system path.
        gz_env['GAZEBO_MODEL_PATH'] = system_models_path + os.pathsep + user_models_path

    # Prevent Hangs
    gz_env['GAZEBO_MODEL_DATABASE_URI'] = 'http://localhost:11111'

    rviz_env = os.environ.copy()
    rviz_env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libpthread.so.0'

    # --- 3. NODES ---
    
    # Gazebo Server 
    # FIXED: Removed '-s libgazebo_ros_state.so' (It is now in the SDF)
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', 
            '--verbose', 
            '-s', 'libgazebo_ros_factory.so', 
            '-s', 'libgazebo_ros_init.so',
            LaunchConfiguration('world_path')
        ],
        output='screen',
        env=gz_env 
    )

    # Gazebo Client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'], 
        output='screen', 
        env=gz_env 
    )

    # RViz
    rviz_action = ExecuteProcess(
        cmd=['/opt/ros/humble/bin/rviz2', '-d', LaunchConfiguration('rviz_config')],
        output='screen', 
        env=rviz_env
    )

    # Relay Node
    relay_node = Node(
        package='gazebo_actor_spawner',
        executable='actor_relay',
        output='screen',
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_path'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        gazebo_server,
        gazebo_client,
        rviz_action,
        relay_node,
        RegisterEventHandler(OnProcessExit(target_action=gazebo_client, on_exit=[EmitEvent(event=Shutdown())]))
    ])