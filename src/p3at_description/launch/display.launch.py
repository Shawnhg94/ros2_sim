# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    description_file = os.path.join(get_package_share_directory("p3at_description"), "urdf", "pioneer1.urdf")
    world_file = os.path.join(get_package_share_directory("p3at_description"), "world", "empty.sdf")
    # cartographer
    cartographer_config_dir = os.path.join(get_package_share_directory('p3at_description'), 'config', 'cartographer')
    configuration_basename = '2d.lua' 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description = ParameterValue(Command(['xacro ', description_file]), value_type=str)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description},
        ]
    )
 
    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory("p3at_description"), 'rviz', 'display.rviz')],
        # parameters=[{"use_sim_time" : True}]
    )

    sdf_world = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        name="create_world",
        output="both"
    )

    robot = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create", "-file", description_file, "-z", "0.2"],
        name="spawn robot",
        output="both"
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                   "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                   "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"]
    )

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )


    cartographer_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')

    cartographer_node =  Node(
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
            )
    cartographer_occupancy_grid_node =  Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': True},
                {'resolution': 0.05}],
            )


    return LaunchDescription([
        rviz_launch_arg,
        cartographer_arg,
        sdf_world,
        robot,
        robot_state_publisher,
        ros_gz_bridge,
        joint_state_pub,
        rviz,
        # robot_steering
        cartographer_node,
        cartographer_occupancy_grid_node,

    ])