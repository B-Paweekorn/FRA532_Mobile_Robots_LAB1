#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_dir = os.path.join(get_package_share_directory("carver_navigation"), "maps")
    map_file = LaunchConfiguration(
        "map", default=os.path.join(map_dir, "vmegarover_samplemap.yaml")
    )

    param_dir = os.path.join(get_package_share_directory("carver_navigation"), "config")
    param_file = LaunchConfiguration(
        "params", default=os.path.join(param_dir, "navigation_param.yaml")
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )
    rviz_config_dir = os.path.join(
        get_package_share_directory("carver_navigation"), "rviz"
    )
    rviz_config_file = os.path.join(rviz_config_dir, "navigation.rviz")

    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_relay",
        output="screen",
        parameters=[
            {"input_topic": "/cmd_vel", "output_topic": "/diff_cont/cmd_vel_unstamped"}
        ],
    )

    map_arg = DeclareLaunchArgument(
        "map", default_value=map_file, description="Full path to map file to load"
    )

    param_arg = DeclareLaunchArgument(
        "params",
        default_value=param_file,
        description="Full path to param file to load",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, "/bringup_launch.py"]),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": param_file,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    description = LaunchDescription()
    description.add_action(map_arg)
    description.add_action(param_arg)
    description.add_action(nav2)
    description.add_action(rviz)
    # description.add_action(cmd_vel_relay)
    return description
