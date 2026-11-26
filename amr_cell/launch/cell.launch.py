"""Cliente de la celda (PLC) por MQTT, opcionalmente con el PLC simulado.

    ros2 launch amr_cell cell.launch.py [broker_host:=<ip>] [cell_id:=st1]
    ros2 launch amr_cell cell.launch.py sim:=true      # + cell_sim en el mismo broker
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration('broker_host')
    port = LaunchConfiguration('broker_port')
    cell = LaunchConfiguration('cell_id')
    return LaunchDescription([
        DeclareLaunchArgument('broker_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('broker_port', default_value='18883'),
        DeclareLaunchArgument('cell_id', default_value='st1'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: lanza también el PLC simulado'),
        Node(package='amr_cell', executable='cell_client', name='cell_client', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('amr_cell'), 'config',
                                               'cell.yaml']),
                         {'broker_host': host,
                          'broker_port': ParameterValue(port, value_type=int),
                          'cell_id': cell}]),
        Node(package='amr_cell', executable='cell_sim', name='cell_sim', output='screen',
             arguments=['--host', host, '--port', port,
                        '--prefix', ['omrover/cell/', cell]],
             condition=IfCondition(LaunchConfiguration('sim'))),
    ])
