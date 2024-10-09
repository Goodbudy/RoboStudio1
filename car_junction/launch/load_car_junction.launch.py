import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set the world file name
    world_file_name = 'car_1_junction22_modified_cleaned.world'
    
    # Get the path to your world file
    pkg_share = get_package_share_directory('Sprint2')
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Path to the world file'),

        # Start Gazebo server with the specified world
        Node(
            package='gazebo_ros',
            executable='gzserver',  # Use executable name without full path
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '--verbose', world_path]),

        # Start Gazebo client
        Node(
            package='gazebo_ros',
            executable='gzclient',  # Use executable name without full path
            output='screen'),
    ])

