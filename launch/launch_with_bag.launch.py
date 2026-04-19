#!/usr/bin/python3
# -- coding: utf-8 --**

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    config_file_dir = os.path.join(get_package_share_directory("gs-lio"), "config")
    foxglove_bridge_dir = get_package_share_directory("foxglove_bridge")
    #Load parameters
    config_cmd = os.path.join(config_file_dir, "cfg.yaml")

    config_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_cmd,
        description='Full path to the ROS2 parameters file to use for livo nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    params_file = LaunchConfiguration('params_file')

    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(foxglove_bridge_dir, "launch", "foxglove_bridge_launch.xml")
        ),
        launch_arguments={
            'address': '0.0.0.0',
            'port': '8765'
        }.items()
    )

    return LaunchDescription([
        config_arg,
        use_respawn_arg,
        foxglove_bridge_launch,
        Node(
            package="gs-lio",
            executable="lio_node",
            name="lio_node",
            parameters=[
                params_file
            ],
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_camera",
            arguments=["0.0403191", "-0.0284392", "-0.0382924", "-0.3533328", "0.3513514", "-0.6145922", "0.611542", "imu_link", "camera_link"],
            output="screen"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher_lidar",
            arguments=["-0.011", "-0.02329", "0.04412", "0", "0", "0", "1", "imu_link", "lidar_link"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', "/ros2_ws/src/gs-lio/bags/dric-lab"
            ],
            output='screen',
            shell=True,
        ),
        Node(
            package="gs-lio",
            executable="triangle_splatting",
            name="triangle_splatting",
            output="screen"
        )
    ])