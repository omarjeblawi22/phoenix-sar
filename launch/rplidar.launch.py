from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # The RPLIDAR serial port can be overridden at launch time.
    # On the Pi, run: ls /dev/ttyUSB* /dev/ttyACM*
    # RPLIDAR A1 usually appears as /dev/ttyUSB0.
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLIDAR (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )

    return LaunchDescription([
        serial_port_arg,
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])
