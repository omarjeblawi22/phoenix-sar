# slam_launch.py
# One-command launcher for Stage 2: SLAM mapping.
# Starts the robot (RSP + ros2_control + RPLIDAR) and slam_toolbox in mapping mode.
# Drive the robot around, then save the map with map_saver_cli.

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
        default_value='/dev/ttyUSB0',
        description='RPLIDAR serial port'
    )

    # Bring up robot (RSP + ros2_control + RPLIDAR + twist_mux)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'launch_robot.launch.py')
        ),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port')}.items()
    )

    # SLAM toolbox in async mapping mode
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    return LaunchDescription([
        serial_port_arg,
        launch_robot,
        slam,
    ])
