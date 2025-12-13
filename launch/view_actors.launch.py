import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share_dir = get_package_share_directory('gazebo_actor_spawner')
    
    # --- Arguments ---
    world_path_arg = DeclareLaunchArgument(
        'world_path',
        default_value=os.path.join(pkg_share_dir, 'worlds', 'empty.sdf'),
        description='Path to the Gazebo world file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share_dir, 'rviz', 'actor_view.rviz'),
        description='Path to the RViz configuration file'
    )
    
    urdf_file_path = os.path.join(pkg_share_dir, 'urdf', 'dummy_robot.urdf')
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    # --- Nodes ---
    
    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_path')}.items()
    )
    
    # Robot State Publisher (for dummy robot)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}]
    )
    
    # Gazebo Actor Relay
    actor_relay_node = Node(
        package='gazebo_actor_spawner',
        executable='gazebo_actor_relay',
        name='gazebo_actor_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'planner_frame': 'odom' # This is the "Fixed Frame"
        }]
    )

    return LaunchDescription([
        world_path_arg,
        rviz_config_arg,
        gazebo_launch,
        robot_state_publisher_node,
        rviz_node,
        actor_relay_node
    ])