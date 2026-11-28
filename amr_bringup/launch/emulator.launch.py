"""SIL sin hardware: Mosquitto local + emulador del MCU + bridge + RSP (+ RViz).

    ros2 launch amr_bringup emulator.launch.py
    ros2 launch amr_bringup emulator.launch.py rviz:=true
    ros2 launch amr_bringup emulator.launch.py emulator:=false   # p. ej. firmware native_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('mqtt_prefix')
    return LaunchDescription([
        DeclareLaunchArgument('mqtt_prefix', default_value='omrover/mm01'),
        DeclareLaunchArgument('emulator', default_value='true'),
        # Brazo en rest al encender, como el robot real: q = [pi, -0.05, 0, 0] con
        # arm_sign = -1 y zero_offset = 0 (arm.yaml) -> counts = -q * 16384 / (2 pi)
        DeclareLaunchArgument('arm_init_counts', default_value='-8191,130,0,0',
                              description='counts iniciales de las juntas del emulador'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('battery_volt', default_value='25.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('amr_bringup'), 'launch', 'llc_bridge.launch.py'])),
            launch_arguments={'broker': 'local', 'mqtt_prefix': prefix,
                              'require_arm_calibration': 'false',
                              'arm_calibration_file': 'none',
                              'optical_frames': 'true'}.items()),
        # El emulador arranca después del broker.
        TimerAction(period=1.0, actions=[
            Node(package='amr_mqtt_bridge', executable='mcu_emulator', name='mcu_emulator',
                 output='screen', condition=IfCondition(LaunchConfiguration('emulator')),
                 arguments=['--port', '18883', '--prefix', prefix,
                            '--battery-volt', LaunchConfiguration('battery_volt'),
                            # con '=': un valor que empieza con '-' argparse lo toma por opción
                            ['--arm-init-counts=', LaunchConfiguration('arm_init_counts')]]),
        ]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('amr_description'), 'rviz', 'amr.rviz'])]),
    ])
