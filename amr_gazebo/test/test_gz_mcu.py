"""Lógica del MCU simulado (GzMcu) sin Gazebo: entradas falsas y reloj controlado."""
import math
from types import SimpleNamespace as NS

import pytest

from amr_mqtt_bridge import protocol as p
from amr_mqtt_bridge.arm_units import ArmUnits

gz_mcu = pytest.importorskip('amr_gazebo.gz_mcu')


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def odom(vx=0.0, vy=0.0, wz=0.0, x=0.0, y=0.0):
    v3 = lambda a, b, c: NS(x=a, y=b, z=c)  # noqa: E731
    return NS(twist=NS(twist=NS(linear=v3(vx, vy, 0.0), angular=v3(0.0, 0.0, wz))),
              pose=NS(pose=NS(position=v3(x, y, 0.0))))


@pytest.fixture
def rig():
    clock = Clock()
    sent = {'cmd_vel': [], 'joints': {}}
    units = ArmUnits(16384, 1.0, [0, 0, 0, 0], [1, 1, 1, 1], [-3.2] * 4, [3.2] * 4)
    mcu = gz_mcu.GzMcu(
        sim_now=clock, units=units, finger_travel=0.035,
        send_cmd_vel=lambda *v: sent['cmd_vel'].append(v),
        send_joint=lambda n, q: sent['joints'].__setitem__(n, q),
        dock=(-1.6, 0.0, 0.35), recharge_v_per_s=0.5, boot_id='simboot')
    mcu.on_joints(NS(name=gz_mcu.ARM_JOINTS + gz_mcu.FINGER_JOINTS,
                     position=[0.0, 0.0, 0.0, 0.0, 0.035, 0.035], velocity=[0.0] * 6))
    return NS(mcu=mcu, clock=clock, sent=sent)


def run(rig, seconds, dt=0.005, meas=None):
    for _ in range(int(round(seconds / dt))):
        rig.clock.t += dt
        if meas is not None:
            rig.mcu.on_odom(meas())
        rig.mcu.step(dt)


def test_uses_sim_time_and_watchdog(rig):
    m = rig.mcu
    run(rig, 0.01)
    cmd = p.encode_command('cmd/vel', 'simboot', 1, vx=0.3, vy=0.0, wz=0.0, valid_ms=250)
    m.handle('cmd/vel', cmd)
    run(rig, 0.2)
    assert rig.sent['cmd_vel'][-1][0] == pytest.approx(0.2, abs=0.02)   # rampa 1 m/s^2, envío a 50 Hz
    run(rig, 0.4)                                                        # expira valid_ms
    assert m.wd_tripped and rig.sent['cmd_vel'][-1][0] < 0.1
    assert p.decode_telemetry('llc/status', m.telemetry('llc/status'))['t_ms'] == m.now_ms()
    assert m.now_ms() == int(rig.clock.t * 1000)


def test_odometry_integrates_measured_twist_and_reset(rig):
    m = rig.mcu
    run(rig, 0.01)
    run(rig, 2.0, meas=lambda: odom(vx=0.5))
    data = p.decode_telemetry('llc/odom', m.telemetry('llc/odom'))
    assert data['x'] == pytest.approx(1.0, abs=0.01) and data['vx'] == pytest.approx(0.5)
    reset = p.encode_command('cmd/odom_reset', 'simboot', 2, x=5.0, y=1.0, yaw=0.5)
    assert p.decode_telemetry('llc/ack', m.handle('cmd/odom_reset', reset)[0][1])['result'] == 'ok'
    run(rig, 1.0, meas=lambda: odom(vx=0.5))
    data = p.decode_telemetry('llc/odom', m.telemetry('llc/odom'))
    assert data['x'] == pytest.approx(5.0 + 0.5 * math.cos(0.5), abs=0.01)
    assert data['y'] == pytest.approx(1.0 + 0.5 * math.sin(0.5), abs=0.01)
    wheels = p.decode_telemetry('llc/wheels', m.telemetry('llc/wheels'))
    assert wheels['vel'][0] == pytest.approx(-math.sin(math.pi / 4) * 0.5 / 0.05, rel=1e-6)


def test_arm_goal_streams_joint_setpoints(rig):
    m = rig.mcu
    run(rig, 0.01)
    m.handle('cmd/arm', p.encode_command('cmd/arm', 'simboot', 3, op='enable'))
    goal = p.encode_command('cmd/arm', 'simboot', 4, op='goal', pos=[4096, 0, 0, -2048],
                            acc=[200.0] * 4, dec=[200.0] * 4)
    assert p.decode_telemetry('llc/ack', m.handle('cmd/arm', goal)[0][1])['result'] == 'ok'
    run(rig, 0.1)
    mid = rig.sent['joints']['arm_joint_1']
    assert 0.0 < mid < math.pi / 2                       # trayectoria, no salto
    run(rig, 3.0)
    assert rig.sent['joints']['arm_joint_1'] == pytest.approx(math.pi / 2, abs=1e-3)
    assert rig.sent['joints']['arm_joint_4'] == pytest.approx(-math.pi / 4, abs=1e-3)
    # gz alcanza la consigna -> in_pos
    m.on_joints(NS(name=gz_mcu.ARM_JOINTS, position=[math.pi / 2, 0, 0, -math.pi / 4],
                   velocity=[0.0] * 4))
    arm = p.decode_telemetry('llc/arm', m.telemetry('llc/arm'))
    assert arm['pos'] == [4096, 0, 0, -2048] and all(arm['in_pos'])


def test_gripper_grasp_when_finger_stalls(rig):
    m = rig.mcu
    run(rig, 0.01)
    m.handle('cmd/gripper', p.encode_command('cmd/gripper', 'simboot', 5, pos=1.0, max_cur=0.8))
    run(rig, 0.05)
    assert rig.sent['joints']['gripper_finger_joint'] == pytest.approx(0.0)
    # el dedo se detiene en 0.015 m (pieza de 3 cm entre los dedos)
    m.on_joints(NS(name=gz_mcu.FINGER_JOINTS, position=[0.015, 0.015], velocity=[0.0, 0.0]))
    run(rig, 0.5)
    arm = p.decode_telemetry('llc/arm', m.telemetry('llc/arm'))
    assert arm['grip']['grasped'] is True
    assert arm['grip']['pos'] == pytest.approx(1 - 0.015 / 0.035, abs=1e-6)
    assert arm['grip']['cur'] > 0.7


def test_battery_override_and_dock_recharge(rig):
    m = rig.mcu
    run(rig, 0.01)
    m.override_battery(21.0)
    run(rig, 1.0, meas=lambda: odom(x=-1.6, y=0.1))       # quieto en el dock
    assert m.battery == pytest.approx(21.5, abs=0.02)
    run(rig, 1.0, meas=lambda: odom(x=2.0, y=0.0))        # lejos: solo descarga
    assert m.battery < 21.5


def test_gripper_closing_on_air_is_not_a_grasp(rig):
    m = rig.mcu
    run(rig, 1.0)                                         # dedo en reposo (abierto) 1 s
    m.handle('cmd/gripper', p.encode_command('cmd/gripper', 'simboot', 6, pos=1.0, max_cur=0.8))
    run(rig, 0.1)
    assert m.grasped is False                             # recién comandado: no hay atasco
    for f in (0.025, 0.015, 0.005, 0.0):                  # el dedo cierra hasta el tope
        m.on_joints(NS(name=gz_mcu.FINGER_JOINTS, position=[f, f], velocity=[0.0, 0.0]))
        run(rig, 0.1)
    run(rig, 1.0)
    arm = p.decode_telemetry('llc/arm', m.telemetry('llc/arm'))
    assert arm['grip']['grasped'] is False and arm['grip']['pos'] == pytest.approx(1.0)
