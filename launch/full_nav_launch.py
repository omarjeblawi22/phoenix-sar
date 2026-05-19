# full_nav_launch.py
# One-command launcher for Stage 3: Autonomous navigation.
# Starts robot + RPLIDAR + AMCL localization + Nav2 navigation stack.
#
# Usage:
#   ros2 launch articubot_one full_nav_launch.py map:=/home/phoenix/ros2_ws/maps/my_map.yaml
#
# The 'map' argument must point to the .yaml file saved by map_saver_cli.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = 'articubot_one'
    pkg_dir = get_package_share_directory(package_name)

    # Default is /dev/ttyUSB1 because in this setup:
    #   ESP32   = /dev/ttyUSB0  (configured in ros2_control.xacro, not this arg)
    #   RPLIDAR = /dev/ttyUSB1  (this arg)
    # Ports can swap after unplug/replug. Always confirm with:
    #   ls /dev/ttyUSB* /dev/ttyACM*
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='RPLIDAR A1 serial port (NOT the ESP32 port)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file'
    )

    # Bring up robot (RSP + ros2_control + RPLIDAR + twist_mux)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'launch_robot.launch.py')
        ),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()
    )

    # Map server + AMCL localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
        }.items()
    )

    # Nav2 navigation stack (controller, planner, bt_navigator, etc.)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        serial_port_arg,
        map_arg,
        launch_robot,
        localization,
        navigation,
    ])
