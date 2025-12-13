import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share_dir = get_package_share_directory('gazebo_actor_spawner')

    # --- 1. Define Environments ---

    # Gazebo Client Environment (The "Sanitized" Zone)
    # We explicitly strip variables that cause Ogre/Gazebo to crash.
    gz_client_env = os.environ.copy()
    gz_client_env['QT_QPA_PLATFORM'] = 'xcb' # Force X11
    if 'LD_PRELOAD' in gz_client_env:
        del gz_client_env['LD_PRELOAD']      # Ensure RViz fix doesn't poison Gazebo
    if 'WAYLAND_DISPLAY' in gz_client_env:
        del gz_client_env['WAYLAND_DISPLAY'] # Prevent Ogre from seeing Wayland

    # RViz Environment (The "Patched" Zone)
    # We explicitly inject the library RViz needs to fix the symbol lookup error.
    rviz_env = os.environ.copy()
    rviz_env['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libpthread.so.0'

    # --- 2. Launch Actions ---

    # A. Gazebo Server (Headless Physics)
    # We use the standard launch file but set gui=false. 
    # gzserver is robust and doesn't need the graphical environment hacks.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_path'),
            'gui': 'false' # We launch the client manually below
        }.items()
    )

    # B. Gazebo Client (Manual GUI Launch)
    # This runs gzclient with our sanitized environment.
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'], 
        output='screen',
        env=gz_client_env
    )

    # C. RViz (Manual Launch)
    # This runs RViz with the patched LD_PRELOAD environment.
    rviz_action = ExecuteProcess(
        cmd=[
            '/opt/ros/humble/bin/rviz2',
            '-d', LaunchConfiguration('rviz_config'),
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen',
        env=rviz_env
    )

    # --- 3. Lifecycle Management ---
    
    # Ensure that if the user closes the Gazebo window, the whole script stops
    # (Mimics standard Gazebo behavior)
    shutdown_on_gz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_client,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_path',
            description='Path to the SDF world file'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share_dir, 'rviz', 'config.rviz'),
            description='Path to RViz config'
        ),
        gazebo_server,
        gazebo_client,
        rviz_action,
        shutdown_on_gz_exit
    ])