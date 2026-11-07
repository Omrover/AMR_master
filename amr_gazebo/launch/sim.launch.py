"""Gemelo digital: Gazebo Harmonic + MCU simulado (MQTT) + bridge real + RSP.

    ros2 launch amr_gazebo sim.launch.py                 # headless (servidor, sin GUI)
    ros2 launch amr_gazebo sim.launch.py gui:=true rviz:=true
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import xacro
from ament_index_python.packages import get_package_share_directory


def spawn_robot(context):
    share = get_package_share_directory('amr_gazebo')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'amr_sim.urdf.xacro'),
                              mappings={'optical_frames': 'true',
                                        'gripper_mimic': 'false'}).toxml()
    path = os.path.join(tempfile.gettempdir(), 'omrover_mm_sim.urdf')
    with open(path, 'w') as f:
        f.write(urdf)
    return [Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'omrover_mm', '-file', path,
                            '-x', LaunchConfiguration('x').perform(context),
                            '-y', LaunchConfiguration('y').perform(context),
                            '-z', '0.01'])]


def gz_args(context):
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(get_package_share_directory('amr_gazebo'), 'worlds', world)
    headless = LaunchConfiguration('gui').perform(context).lower() != 'true'
    if LaunchConfiguration('render').perform(context).lower() != 'true':
        # Sin el sistema Sensors no se crea el motor de render: LiDAR y cámara quedan
        # inactivos, pero física, IMU, odometría y brazo funcionan (CI, equipos sin GPU).
        with open(world) as f:
            sdf = f.read()
        start = sdf.find('<plugin filename="gz-sim-sensors-system"')
        end = sdf.find('</plugin>', start) + len('</plugin>')
        world = os.path.join(tempfile.gettempdir(), 'amr_world_norender.sdf')
        with open(world, 'w') as f:
            f.write(sdf[:start] + sdf[end:] if start >= 0 else sdf)
    render = 'norender' not in world
    args = f'-r {world}' + (' -s' if headless else '')
    if headless and render:
        args += ' --headless-rendering'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    gazebo_pkg = FindPackageShare('amr_gazebo')
    bridge_pkg = FindPackageShare('amr_mqtt_bridge')
    prefix = LaunchConfiguration('mqtt_prefix')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse.sdf'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('render', default_value='true',
                              description='false: sin LiDAR/cámara (no requiere GPU)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('mqtt_prefix', default_value='omrover/mm01'),
        DeclareLaunchArgument('battery_volt', default_value='25.0'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(filter(None, [
            os.path.dirname(get_package_share_directory('amr_description')),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))),
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=gz_args),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution(
                 [gazebo_pkg, 'config', 'ros_gz_bridge.yaml'])}]),
        TimerAction(period=3.0, actions=[OpaqueFunction(function=spawn_robot)]),
        # Broker local + bridge real + robot_state_publisher (misma pila que el robot).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('amr_bringup'), 'launch', 'llc_bridge.launch.py'])),
            launch_arguments={'broker': 'local', 'mqtt_prefix': prefix,
                              'optical_frames': 'true'}.items()),
        TimerAction(period=5.0, actions=[
            Node(package='amr_gazebo', executable='gz_mcu', name='gz_mcu', output='screen',
                 parameters=[PathJoinSubstitution([bridge_pkg, 'config', 'arm.yaml']),
                             {'mqtt_prefix': prefix,
                              'battery_volt': ParameterValue(LaunchConfiguration('battery_volt'),
                                                             value_type=float)}]),
        ]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([gazebo_pkg, 'rviz', 'sim.rviz'])]),
    ])
