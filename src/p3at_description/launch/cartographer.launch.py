import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your custom Lua configuration file
    cartographer_config_dir = os.path.join(get_package_share_directory('p3at_description'), 'config', 'cartographer') # Replace 'my_robot_package'
    configuration_basename = '2d.lua' # Your Lua filename

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            # Add remappings if your topics are non-standard for Cartographer
            # remappings=[
            # ('scan', '/my_robot/laser_scan'),
            # ('odom', '/my_robot/odometry'),
            # ('imu', '/my_robot/imu_data')
            # ]
            ),

        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': True},
                {'resolution': 0.05}],
            ),
        
        ])