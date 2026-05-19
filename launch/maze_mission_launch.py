# maze_mission_launch.py
#
# PHOENIX SAR — Maze Navigation + Target Detection Mission
#
# What this does:
#   1. Starts robot bringup (RSP + ros2_control + RPLIDAR + twist_mux)
#   2. Starts SLAM toolbox (live mapping while navigating)
#   3. Starts Nav2 (autonomous path planning + execution)
#   4. Starts camera_detector_node (publishes /phoenix/target_detected at 10 Hz)
#   5. Starts phoenix_explorer (frontier exploration — stops when camera detects target)
#
# When the target is detected:
#   - Robot stops immediately
#   - Map is saved to map_save_path.*
#   - Shortest path (start → target) is computed and published to /phoenix/shortest_path
#
# Usage:
#   ros2 launch articubot_one maze_mission_launch.py \
#     serial_port:=/dev/ttyUSB1 \
#     model_path:=/home/phoenix/model/target_classifier_int8.tflite \
#     metadata_path:=/home/phoenix/model/metadata.json
#
# Optional args:
#   camera_index     (default 0)
#   map_save_path    (default /home/phoenix/ros2_ws/maps/maze_mission)
#   explore_duration (default 3600 — 1-hour fallback timeout)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# libcamera Python bindings are at a non-standard path on Ubuntu 24.04 Pi 5.
_LIBCAMERA_SITE = '/usr/lib/aarch64-linux-gnu/python3.12/site-packages'
_PYTHONPATH = f"{_LIBCAMERA_SITE}:{os.environ.get('PYTHONPATH', '')}"


def generate_launch_description():

    package_name = 'articubot_one'
    pkg_dir = get_package_share_directory(package_name)

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB1',
        description='RPLIDAR serial port')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/phoenix/model/target_classifier_int8.tflite',
        description='Path to TFLite model')

    metadata_path_arg = DeclareLaunchArgument(
        'metadata_path',
        default_value='/home/phoenix/model/metadata.json',
        description='Path to model metadata.json')

    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='OpenCV camera index for Pi Camera (usually 0)')

    map_save_path_arg = DeclareLaunchArgument(
        'map_save_path',
        default_value='/home/phoenix/ros2_ws/maps/maze_mission',
        description='File path prefix for saved map (.pgm and .yaml)')

    explore_duration_arg = DeclareLaunchArgument(
        'explore_duration', default_value='3600.0',
        description='Fallback timeout in seconds if target is never found')

    # ----------------------------------------------------------------
    # SLAM + Nav2 (robot bringup + SLAM toolbox + navigation stack)
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
    # Camera detector node
    # Starts after 5 s to let the camera hardware initialise.
    # Publishes /phoenix/target_detected at 10 Hz.
    # ----------------------------------------------------------------
    camera_detector = Node(
        package='articubot_one',
        executable='camera_detector',
        name='camera_detector',
        output='screen',
        additional_env={'PYTHONPATH': _PYTHONPATH},
        parameters=[{
            'model_path':    LaunchConfiguration('model_path'),
            'metadata_path': LaunchConfiguration('metadata_path'),
            'camera_index':  LaunchConfiguration('camera_index'),
        }]
    )
    delayed_camera = TimerAction(period=5.0, actions=[camera_detector])

    # ----------------------------------------------------------------
    # Phoenix explorer (frontier exploration with camera stop)
    # Delayed 25 s so SLAM has a first map and Nav2 is fully active.
    # ----------------------------------------------------------------
    explorer = Node(
        package='articubot_one',
        executable='phoenix_explorer',
        name='phoenix_explorer',
        output='screen',
        parameters=[{
            'explore_duration':  LaunchConfiguration('explore_duration'),
            'map_save_path':     LaunchConfiguration('map_save_path'),
            'fixed_frame':       'map',
            'robot_frame':       'base_link',
            'goal_timeout_s':    15.0,
            'min_goal_dist_m':    0.3,
            'max_goal_dist_m':    1.5,
            'min_frontier_cells': 3,
            'spin_scan':          True,
            'spin_yaw':           6.28,  # 360° panoramic scan at each frontier
        }]
    )
    delayed_explorer = TimerAction(period=25.0, actions=[explorer])

    # ----------------------------------------------------------------
    # Web dashboard  (http://<pi-ip>:8080)
    # Starts after 3 s — subscribes to topics, no hard dependencies.
    # Requires:  pip3 install flask
    # ----------------------------------------------------------------
    dashboard = Node(
        package='articubot_one',
        executable='phoenix_dashboard',
        name='phoenix_dashboard',
        output='screen',
    )
    delayed_dashboard = TimerAction(period=3.0, actions=[dashboard])

    return LaunchDescription([
        serial_port_arg,
        model_path_arg,
        metadata_path_arg,
        camera_index_arg,
        map_save_path_arg,
        explore_duration_arg,
        slam_nav,
        delayed_camera,
        delayed_explorer,
        delayed_dashboard,
    ])
