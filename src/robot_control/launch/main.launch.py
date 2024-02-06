#!/usr/bin/python3

import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('robot_control'), 'launch')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, "/simulation.launch.py"])
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, "/navigation.launch.py"])
    )

    odom_publisher = Node(
        package="robot_control",
        executable="odom_publisher.py",
        parameters=[{"use_sim_time": True}]
    )

    velocity_controller = Node(
        package="robot_control",
        executable="velocity_controller.py",
        parameters=[{"use_sim_time": True}],
        remappings={("/cmd_vel", "/cmd_vel_nav2")}
    )

    description = LaunchDescription()
    description.add_action(simulation)
    description.add_action(navigation)
    # description.add_action(velocity_controller)
    description.add_action(odom_publisher)
    return description
