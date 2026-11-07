"""Visualiza el robot en RViz con robot_state_publisher y (opcional) joint_state_publisher_gui."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('amr_description')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    optical = LaunchConfiguration('optical_frames')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'amr.urdf.xacro']),
                 ' optical_frames:=', optical]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui para mover las juntas'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('optical_frames', default_value='true',
                              description='Agrega los frames ópticos de la cámara'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'amr.rviz'])]),
    ])
