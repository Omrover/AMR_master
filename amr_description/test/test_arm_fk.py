"""La cadena arm_base_link -> tool0 de la URDF (mallas y frames del CAD) debe reproducir
la tabla DH de AMR_arm más la herramienta (amr_arm_control/kinematics.py)."""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from urdf_tools import UrdfChain, process  # noqa: E402

L1 = 0.02045 + 0.0772
L2 = 0.4
L3 = 0.095
L4 = 0.24525
C = math.sqrt(L3 ** 2 + L4 ** 2)
PHI = math.acos(L4 / C)
GAMMA = math.pi + PHI
# tool0 en el frame 4 (raíz de los dedos del CAD)
T0_X, T0_Y, T0_YAW = -0.17479, -0.05323, -2.772
ALONG = T0_X * math.cos(T0_YAW) + T0_Y * math.sin(T0_YAW)
PERP = -T0_X * math.sin(T0_YAW) + T0_Y * math.cos(T0_YAW)


def dh(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


def fk_dh(q1, q2, q3, q4):
    """Tabla DH de AMR_arm (estándar) + tool0."""
    tool = np.eye(4)
    tool[0, 3], tool[1, 3] = ALONG, PERP
    return (dh(q1, L1, 0, math.pi / 2) @ dh(q2, 0, L2, 0)
            @ dh(q3 - PHI, 0, -C, 0) @ dh(q4 + GAMMA, 0, 0, 0) @ tool)


@pytest.fixture(scope='module')
def chain():
    return UrdfChain(process())


def test_zero_pose(chain):
    t = chain.transform('arm_base_link', 'tool0')
    assert np.allclose(t, fk_dh(0, 0, 0, 0), atol=1e-4)


def test_random_poses_match_dh(chain):
    rng = np.random.default_rng(7)
    for _ in range(500):
        q = rng.uniform(-math.pi, math.pi, 4)
        t_urdf = chain.transform('arm_base_link', 'tool0', {
            'arm_joint_1': q[0], 'arm_joint_2': q[1],
            'arm_joint_3': q[2], 'arm_joint_4': q[3]})
        assert np.allclose(t_urdf, fk_dh(*q), atol=1e-4)


def test_fingers_touch_when_closed_and_open_symmetric(chain):
    """Dedos del CAD: con 0 se tocan sobre el plano del brazo; abren a lo largo de z."""
    left = chain.transform('tool0', 'gripper_finger_left_link')
    right = chain.transform('tool0', 'gripper_finger_right_link')
    assert left[2, 3] > 0 > right[2, 3]
    opened = chain.transform('tool0', 'gripper_finger_left_link', {'gripper_finger_joint': 0.05})
    assert opened[2, 3] - left[2, 3] == pytest.approx(0.05, abs=1e-9)
