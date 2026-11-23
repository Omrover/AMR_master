"""Driver del RPLIDAR A1M8 (rplidar_ros) publicando /scan en laser_link.

    ros2 launch amr_bringup lidar.launch.py [serial_port:=/dev/rplidar]

Requiere `sudo apt install ros-jazzy-rplidar-ros` y la regla udev
config/udev/99-rplidar.rules (crea /dev/rplidar para el CP2102 del A1):

    sudo cp config/udev/99-rplidar.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger

La pose del LiDAR en el robot (lidar_x/y/z y yaw) está en amr_description/urdf/
params.xacro: verificar con un objeto al frente del robot que aparezca en +x en RViz.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar',
             output='screen',
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': LaunchConfiguration('serial_port'),
                 'serial_baudrate': 115200,      # A1M8
                 'frame_id': LaunchConfiguration('frame_id'),
                 'inverted': False,
                 'angle_compensate': True,
                 'scan_mode': 'Standard',
             }]),
    ])
