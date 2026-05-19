# explore_launch.py
#
# PHOENIX SAR — Full Autonomous Exploration Launch
#
# Starts everything needed for the 60-second exploration mission:
#   1. Robot bringup  (RSP + ros2_control + RPLIDAR + twist_mux)
#   2. SLAM toolbox   (mapping mode — builds map in real time)
#   3. Nav2           (autonomous navigation against the live SLAM map)
#   4. Phoenix Explorer node (frontier exploration + timed stop + return path)
#
# Usage:
#   ros2 launch articubot_one explore_launch.py \
#     serial_port:=/dev/ttyUSB1 \
#     explore_duration:=60.0 \
#     map_save_path:=/home/phoenix/ros2_ws/maps/exploration_run
#
# The explorer node is delayed by 20 s to give SLAM and Nav2 time to initialise.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'articubot_one'
    pkg_dir = get_package_share_directory(package_name)

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='RPLIDAR A1 serial port (NOT the ESP32 port)'
    )

    explore_duration_arg = DeclareLaunchArgument(
        'explore_duration',
        default_value='60.0',
        description='Exploration duration in seconds before stopping'
    )

    map_save_path_arg = DeclareLaunchArgument(
        'map_save_path',
        default_value='/home/phoenix/ros2_ws/maps/exploration_run',
        description='Path prefix for the saved map files (.pgm and .yaml)'
    )

    # ----------------------------------------------------------------
    # Existing combined launch: robot + SLAM + Nav2
    # ----------------------------------------------------------------
    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'slam_nav_launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )

    # ----------------------------------------------------------------
    # Phoenix Explorer node
    # Delayed 20 s so SLAM has time to receive first scans and Nav2
    # lifecycle nodes finish activating.
    # ----------------------------------------------------------------
    explorer_node = Node(
        package='articubot_one',
        executable='phoenix_explorer',
        name='phoenix_explorer',
        output='screen',
        parameters=[{
            'explore_duration':    LaunchConfiguration('explore_duration'),
            'map_save_path':       LaunchConfiguration('map_save_path'),
            'fixed_frame':         'map',
            'robot_frame':         'base_link',
            'goal_timeout_s':      12.0,
            'min_goal_dist_m':      1.0,
            'max_goal_dist_m':      5.0,
            'min_frontier_cells':   5,
        }]
    )

    delayed_explorer = TimerAction(period=20.0, actions=[explorer_node])

    return LaunchDescription([
        serial_port_arg,
        explore_duration_arg,
        map_save_path_arg,
        slam_nav,
        delayed_explorer,
    ])
