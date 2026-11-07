"""El modelo de simulación (xacro + plugins) procesa y, si gz está disponible, convierte a SDF."""
import os
import shutil
import subprocess

import pytest

xacro = pytest.importorskip('xacro')
ament = pytest.importorskip('ament_index_python.packages')


def sim_urdf():
    share = ament.get_package_share_directory('amr_gazebo')
    return xacro.process_file(os.path.join(share, 'urdf', 'amr_sim.urdf.xacro'),
                              mappings={'optical_frames': 'true',
                                        'gripper_mimic': 'false'}).toxml()


def test_plugins_and_sensors_present():
    urdf = sim_urdf()
    for item in ('gz::sim::systems::VelocityControl', 'gz::sim::systems::OdometryPublisher',
                 'gz::sim::systems::JointStatePublisher', 'type="gpu_lidar"', 'type="imu"',
                 'type="rgbd_camera"', '/sim/arm_joint_4/cmd_pos',
                 '/sim/gripper_finger_joint/cmd_pos'):
        assert item in urdf, item
    assert urdf.count('JointPositionController') == 6
    assert '<mimic' not in urdf


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz no instalado')
def test_converts_to_sdf(tmp_path):
    path = tmp_path / 'amr.urdf'
    path.write_text(sim_urdf())
    result = subprocess.run(['gz', 'sdf', '-p', str(path)], capture_output=True, text=True,
                            timeout=60)
    assert result.returncode == 0, result.stderr
    sdf = result.stdout
    for name in ('rplidar_a1', 'bno080', 'd435i', 'arm_joint_1', 'gripper_finger_joint'):
        assert name in sdf, name
