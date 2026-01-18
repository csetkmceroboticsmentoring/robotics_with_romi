#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Get the path to the SDF file
    sdf_file = os.path.join(
        '/ros2_ws/exploration/romi_robot_demos/romi_gazebo',
        'tugbot_depot.sdf',
        #'romi_world.sdf'
    )
    
    # Launch Gazebo with the world file (similar to working example)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {sdf_file}'
        }.items(),
    )
    
    # Bridge for cmd_vel and odometry (like the working example)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/romi/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/romi/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        parameters=[{
            'qos_overrides./model/romi.subscriber.reliability': 'reliable'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        bridge,
    ])

