"""Bridge MQTT con Tlaloc + robot_state_publisher (robot real o emulador).

    ros2 launch amr_bringup llc_bridge.launch.py                 # broker ya corriendo
    ros2 launch amr_bringup llc_bridge.launch.py broker:=real     # lanza mosquitto amr.conf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (Command, EqualsSubstitution, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('amr_bringup')
    bridge_pkg = FindPackageShare('amr_mqtt_bridge')
    description = FindPackageShare('amr_description')
    broker = LaunchConfiguration('broker')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([description, 'urdf', 'amr.urdf.xacro']),
                 ' optical_frames:=', LaunchConfiguration('optical_frames')]),
        value_type=str)

    def mosquitto(conf, name):
        return ExecuteProcess(
            cmd=['mosquitto', '-c', PathJoinSubstitution([bringup, 'config', 'mosquitto', conf])],
            name=name, output='log',
            condition=IfCondition(EqualsSubstitution(broker, name)))

    return LaunchDescription([
        DeclareLaunchArgument('broker', default_value='none',
                              choices=['none', 'real', 'local'],
                              description='none: usar un broker existente; real: amr.conf '
                                          '(192.168.50.1); local: amr_local.conf (127.0.0.1)'),
        DeclareLaunchArgument('mqtt_prefix', default_value='omrover/mm01'),
        DeclareLaunchArgument('optical_frames', default_value='false',
                              description='true si no corre el driver de la RealSense'),
        mosquitto('amr.conf', 'real'),
        mosquitto('amr_local.conf', 'local'),
        Node(package='amr_mqtt_bridge', executable='bridge', name='amr_mqtt_bridge',
             output='screen',
             parameters=[PathJoinSubstitution([bridge_pkg, 'config', 'bridge.yaml']),
                         PathJoinSubstitution([bridge_pkg, 'config', 'arm.yaml']),
                         {'mqtt_prefix': LaunchConfiguration('mqtt_prefix')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
    ])
