"""Integración: Mosquitto real + cell_sim + cell_client en procesos separados,
con un cliente de la acción en el proceso del test.

Se salta si faltan mosquitto, paho-mqtt, rclpy o amr_interfaces (fuera de la NUC).
"""
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import pytest

pytest.importorskip('paho.mqtt.client')
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('amr_interfaces.action')
MOSQUITTO = shutil.which('mosquitto') or (
    '/usr/sbin/mosquitto' if os.path.exists('/usr/sbin/mosquitto') else None)
pytestmark = pytest.mark.skipif(MOSQUITTO is None, reason='mosquitto no instalado')

from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

from amr_interfaces.action import NotifyCell  # noqa: E402
from amr_interfaces.msg import CellStatus  # noqa: E402

R = NotifyCell.Result


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.fixture(scope='module')
def broker(tmp_path_factory):
    port = free_port()
    conf = tmp_path_factory.mktemp('mosq') / 'test.conf'
    conf.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    proc = subprocess.Popen([MOSQUITTO, '-c', str(conf)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    for _ in range(50):
        try:
            socket.create_connection(('127.0.0.1', port), timeout=0.1).close()
            break
        except OSError:
            time.sleep(0.1)
    yield port
    proc.terminate()
    proc.wait(timeout=5)


def start_sim(port, cell, *extra):
    proc = subprocess.Popen(
        [sys.executable, '-u', '-m', 'amr_cell.cell_sim', '--port', str(port),
         '--prefix', f'omrover/cell/{cell}', *extra],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    deadline = time.time() + 5
    while time.time() < deadline:
        line = proc.stdout.readline()
        if 'listo' in line:
            return proc
    proc.kill()
    raise RuntimeError('cell_sim no arrancó')


def stop(proc, sig=signal.SIGTERM):
    if proc.poll() is None:
        proc.send_signal(sig)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


class Rig:
    """cell_client en su proceso + nodo sonda (cliente de la acción y cell/status)."""

    def __init__(self, port, cell, **params):
        ns = f'/t{cell}'
        args = ['--ros-args', '-r', f'__ns:={ns}', '-p', f'broker_port:={port}',
                '-p', f'cell_id:={cell}', '-p', 'ack_timeout:=0.5', '-p', 'ack_retries:=2',
                '-p', 'ready_timeout:=1.5', '-p', 'done_timeout:=5.0']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
        self.proc = subprocess.Popen([sys.executable, '-u', '-m', 'amr_cell.cell_client', *args],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        self.node = rclpy.create_node(f'probe_{cell}')
        self.status = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(CellStatus, f'{ns}/cell/status',
                                      lambda m: setattr(self, 'status', m), qos)
        self.client = ActionClient(self.node, NotifyCell, f'{ns}/cell/notify_placed')
        assert self.client.wait_for_server(timeout_sec=15.0)

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if cond():
                return True
        return False

    def wait_state(self, state, timeout=5.0):
        return self.spin_until(lambda: self.status is not None and self.status.state == state,
                               timeout)

    def notify(self, piece='pieza_a', wait_done=False, timeout=20.0, cancel_after=None):
        fb = []
        fut = self.client.send_goal_async(NotifyCell.Goal(piece=piece, wait_done=wait_done),
                                          feedback_callback=lambda m: fb.append(m.feedback))
        assert self.spin_until(fut.done, 5.0)
        gh = fut.result()
        assert gh.accepted
        if cancel_after is not None:
            self.spin_until(lambda: False, cancel_after)
            gh.cancel_goal_async()
        res = gh.get_result_async()
        assert self.spin_until(res.done, timeout)
        return res.result().result, fb

    def close(self):
        stop(self.proc, signal.SIGINT)
        self.node.destroy_node()


@pytest.fixture(scope='module')
def ros():
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init()
    yield
    rclpy.shutdown()


def test_ack_and_done(broker, ros):
    sim = start_sim(broker, 'a', '--done-delay', '1.0')
    rig = Rig(broker, 'a')
    try:
        assert rig.wait_state('ready')
        assert rig.status.broker_connected and rig.status.plc_online
        res, fb = rig.notify(wait_done=True)
        assert res.success and res.error_code == R.OK, res.message
        assert [f.stage for f in fb][-1] == 'done'
        assert 'acked' in [f.stage for f in fb]
        # el PLC vuelve a ready y acepta la siguiente pieza con otro seq
        assert rig.wait_state('ready')
        res2, _ = rig.notify(wait_done=False)
        assert res2.success and res2.seq == (res.seq + 1) & 0xFFFFFFFF
    finally:
        rig.close()
        stop(sim)


def test_retry_after_lost_ack(broker, ros):
    sim = start_sim(broker, 'b', '--drop-acks', '1')
    rig = Rig(broker, 'b')
    try:
        assert rig.wait_state('ready')
        res, fb = rig.notify()
        assert res.success, res.message
        assert max(f.attempt for f in fb) == 2      # se reenvió el mismo seq
    finally:
        rig.close()
        stop(sim)


def test_rejected_and_ack_timeout(broker, ros):
    sim = start_sim(broker, 'c', '--reject')
    rig = Rig(broker, 'c')
    try:
        assert rig.wait_state('ready')
        res, _ = rig.notify()
        assert not res.success and res.error_code == R.REJECTED
    finally:
        stop(sim)
    sim = start_sim(broker, 'c', '--drop-acks', '99')
    try:
        assert rig.wait_state('ready')
        t0 = time.time()
        res, fb = rig.notify()
        assert res.error_code == R.ACK_TIMEOUT
        assert max(f.attempt for f in fb) == 3 and time.time() - t0 < 4.0
    finally:
        rig.close()
        stop(sim)


def test_not_ready_offline_and_lwt(broker, ros):
    rig = Rig(broker, 'd')
    try:
        assert rig.spin_until(lambda: rig.status is not None and rig.status.broker_connected,
                              5.0)
        res, _ = rig.notify()                       # sin PLC: nunca hay ready
        assert res.error_code == R.NOT_READY
        sim = start_sim(broker, 'd', '--state', 'busy')
        assert rig.wait_state('busy')
        res, _ = rig.notify()
        assert res.error_code == R.NOT_READY
        stop(sim, signal.SIGKILL)                   # caída sin DISCONNECT: LWT
        assert rig.wait_state('offline', timeout=20.0)
        assert not rig.status.plc_online
    finally:
        rig.close()


def test_done_timeout_and_cancel(broker, ros):
    sim = start_sim(broker, 'e', '--no-done')
    rig = Rig(broker, 'e')
    try:
        assert rig.wait_state('ready')
        res, _ = rig.notify(wait_done=True)
        assert res.error_code == R.DONE_TIMEOUT
        res, _ = rig.notify(wait_done=True, cancel_after=0.8)
        assert res.error_code == R.CANCELED
    finally:
        rig.close()
        stop(sim)


def test_broker_down(ros):
    rig = Rig(free_port(), 'f')
    try:
        res, _ = rig.notify()
        assert res.error_code == R.BROKER_DOWN
    finally:
        rig.close()
