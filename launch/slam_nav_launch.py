# slam_nav_launch.py
#
# Live SLAM + Autonomous Navigation — no saved map required.
#
# Use this when:
#   - You don't have a good saved map yet
#   - You want to build the map and navigate simultaneously
#
# How it works:
#   slam_toolbox runs in mapping mode and publishes /map + map->odom TF.
#   Nav2 uses that live map for global planning without needing AMCL or map_server.
#   As you send goals, the robot navigates AND the map grows.
#
# Trade-off vs saved-map navigation:
#   + No initial pose needed (robot starts at map origin automatically)
#   + Map improves as robot explores
#   - Less accurate localization than AMCL with a complete map
#   - If slam_toolbox loses track, navigation also breaks
#
# Usage:
#   ros2 launch articubot_one slam_nav_launch.py serial_port:=/dev/ttyUSB1
#
# After launching:
#   1. Open RViz, set Fixed Frame = map, add Map + RobotModel + LaserScan
#   2. Send a 2D Goal Pose — robot navigates while building the map
#   3. Optionally save the map when done:
#      ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = 'articubot_one'
    pkg_dir = get_package_share_directory(package_name)

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='RPLIDAR A1 serial port (NOT the ESP32 port)'
    )

    # Robot bringup (RSP + ros2_control + RPLIDAR + twist_mux)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'launch_robot.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )

    # SLAM toolbox — mapping mode (provides /map topic and map->odom TF)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                pkg_dir, 'config', 'mapper_params_online_async.yaml'
            )
        }.items()
    )

    # Nav2 navigation stack only — no map_server, no AMCL.
    # slam_toolbox provides the /map and map->odom transform so Nav2
    # can plan against the live map without needing AMCL for localization.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        serial_port_arg,
        launch_robot,
        slam,
        navigation,
    ])
