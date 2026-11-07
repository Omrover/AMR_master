import math
import shutil
import subprocess
import sys
import pathlib

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from urdf_tools import UrdfChain, process  # noqa: E402

REQUIRED_LINKS = {
    'base_footprint', 'base_link', 'imu_link', 'laser_link', 'tray_link',
    'wheel_fl_link', 'wheel_rl_link', 'wheel_rr_link', 'wheel_fr_link',
    'arm_base_link', 'arm_link_1', 'arm_link_2', 'arm_link_3', 'arm_link_4',
    'tool0', 'gripper_base_link', 'gripper_tcp', 'camera_link',
}


@pytest.mark.parametrize('optical', [False, True])
def test_required_frames(optical):
    chain = UrdfChain(process(optical_frames=optical))
    assert REQUIRED_LINKS <= chain.link_names
    assert ('camera_color_optical_frame' in chain.link_names) == optical


def test_ros2_control_block_optional():
    assert '<ros2_control' not in process()
    xml = process(use_ros2_control=True, ros2_control_hardware='mock')
    assert 'mock_components/GenericSystem' in xml


def test_wheels_x_configuration():
    chain = UrdfChain(process())
    for name, deg in (('fl', 45), ('rl', 135), ('rr', 225), ('fr', 315)):
        j = chain.joints[f'wheel_{name}_link']
        assert j['type'] == 'continuous'
        angle = math.atan2(j['xyz'][1], j['xyz'][0]) % (2 * math.pi)
        assert angle == pytest.approx(math.radians(deg), abs=1e-9)
        assert j['rpy'][2] == pytest.approx(math.radians(deg), abs=1e-9)


def test_optical_frame_convention():
    chain = UrdfChain(process(optical_frames=True))
    t = chain.transform('camera_link', 'camera_color_optical_frame')
    # z óptico = x de camera_link (hacia adelante), x óptico = -y (derecha).
    assert t[:3, 2] == pytest.approx([1, 0, 0], abs=1e-9)
    assert t[:3, 0] == pytest.approx([0, -1, 0], abs=1e-9)


@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf no instalado')
def test_check_urdf(tmp_path):
    urdf = tmp_path / 'amr.urdf'
    urdf.write_text(process(optical_frames=True))
    result = subprocess.run(['check_urdf', str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
