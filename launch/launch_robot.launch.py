import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'articubot_one'

    # Allow the RPLIDAR serial port to be overridden from the command line.
    # Default is /dev/ttyUSB0. Run 'ls /dev/ttyUSB* /dev/ttyACM*' on the Pi to find yours.
    # Default is /dev/ttyUSB1 because in this setup:
    #   ESP32  = /dev/ttyUSB0  (configured in ros2_control.xacro)
    #   RPLIDAR = /dev/ttyUSB1 (this argument)
    # These ports can swap after unplug/replug — always verify with:
    #   ls /dev/ttyUSB* /dev/ttyACM*
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the RPLIDAR A1 (NOT the ESP32 port)'
    )
    serial_port = LaunchConfiguration('serial_port')

    # --- Robot State Publisher ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # --- RPLIDAR ---
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rplidar.launch.py')
        ]),
        launch_arguments={'serial_port': serial_port}.items()
    )

    # --- Twist Mux (multiplexes multiple cmd_vel sources) ---
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )

    # --- Controller Manager ---
    # Build robot_description directly from xacro (avoids race conditions).
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(package_name), 'description', 'robot.urdf.xacro'
        ]),
        ' use_ros2_control:=true sim_mode:=false'
    ])
    robot_description = {'robot_description': robot_description_content}

    controller_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'my_controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        output='screen'
    )

    # Delay controller_manager 3 s to give robot_state_publisher time to start.
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # --- Differential Drive Controller Spawner ---
    # Note: in Jazzy/Humble the executable is 'spawner' (no .py suffix).
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # --- Joint State Broadcaster Spawner ---
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        serial_port_arg,
        rsp,
        rplidar,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
    ])
