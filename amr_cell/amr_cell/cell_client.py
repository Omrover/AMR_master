"""Cliente ROS 2 <-> MQTT de la celda (PLC). Contrato: CELL_CONTRACT.md.

    acción  cell/notify_placed  (amr_interfaces/action/NotifyCell)
    topic   cell/status         (amr_interfaces/msg/CellStatus, transient local)

Hilos: paho actualiza el estado bajo self.lock y despierta a la meta en curso
con threading.Event; la meta corre en el executor multihilo y espera en
intervalos cortos para atender la cancelación. Una sola meta a la vez.
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from amr_interfaces.action import NotifyCell
from amr_interfaces.msg import CellStatus

from amr_cell import protocol as p
from amr_cell.mqtt_support import make_client

STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
R = NotifyCell.Result


class Pending:
    def __init__(self, seq):
        self.seq = seq
        self.ack = None            # (seq, result, msg)
        self.done = None
        self.ack_evt = threading.Event()
        self.done_evt = threading.Event()


class CellClient(Node):
    def __init__(self):
        super().__init__('cell_client')
        d = self.declare_parameter
        d('broker_host', '127.0.0.1')
        d('broker_port', 18883)
        d('username', '')
        d('password', '')
        d('prefix_root', 'omrover/cell')
        d('cell_id', 'st1')
        d('robot_id', 'mm01')
        d('ack_timeout', 3.0)
        d('ack_retries', 3)
        d('ready_timeout', 10.0)
        d('done_timeout', 60.0)
        d('require_ready', True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.cell_id = g('cell_id')
        self.robot_id = g('robot_id')
        self.topics = p.topics(f"{g('prefix_root').rstrip('/')}/{self.cell_id}")
        self.ack_timeout = float(g('ack_timeout'))
        self.ack_retries = int(g('ack_retries'))
        self.ready_timeout = float(g('ready_timeout'))
        self.done_timeout = float(g('done_timeout'))
        self.require_ready = bool(g('require_ready'))

        self.lock = threading.Lock()
        self.connected = False
        self.plc_state = 'unknown'
        self.detail = ''
        self.pending = None
        self.busy = False
        self.seq = (p.initial_seq() - 1) & p.UINT32_MAX
        self.last_status = None

        self.status_pub = self.create_publisher(CellStatus, 'cell/status', STATUS_QOS)
        self.create_timer(0.2, self.publish_status)
        cb = ReentrantCallbackGroup()
        self.action = ActionServer(
            self, NotifyCell, 'cell/notify_placed', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self.client = make_client(
            client_id=f'{self.robot_id}-cell-{self.cell_id}',
            will=(self.topics['robot_status'],
                  p.encode_robot_status(self.robot_id, 'offline'), 1, True),
            username=g('username'), password=g('password'))
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        host, port = g('broker_host'), int(g('broker_port'))
        self.get_logger().info(f'celda {self.cell_id}: MQTT {host}:{port} '
                               f'{self.topics["placed"].rsplit("/robot", 1)[0]}')
        self.client.connect_async(host, port, keepalive=10)
        self.client.loop_start()

    # ------------------------------------------------------------- hilo paho
    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            with self.lock:
                self.detail = f'broker rechazó la conexión (rc={rc})'
            return
        client.subscribe([(self.topics['ack'], 1), (self.topics['done'], 1),
                          (self.topics['plc_status'], 1)])
        client.publish(self.topics['robot_status'],
                       p.encode_robot_status(self.robot_id, 'online'), qos=1, retain=True)
        with self.lock:
            self.connected = True
            self.detail = ''

    def on_disconnect(self, client, userdata, rc):
        with self.lock:
            self.connected = False
            self.plc_state = 'unknown'
            self.detail = 'sin broker'

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == self.topics['plc_status']:
                state = p.decode_plc_status(msg.payload)
                with self.lock:
                    self.plc_state = state
                return
            if msg.topic not in (self.topics['ack'], self.topics['done']):
                return
            is_ack = msg.topic == self.topics['ack']
            item = (p.decode_ack if is_ack else p.decode_done)(msg.payload)
        except p.ProtocolError as e:
            self.get_logger().warning(f'{msg.topic}: mensaje inválido ({e})')
            return
        with self.lock:
            pend = self.pending
            if pend is None or item[0] != pend.seq:
                return                  # ack/done viejo o ajeno: se ignora
            if is_ack:
                pend.ack = item
                pend.ack_evt.set()
            else:
                pend.done = item
                pend.done_evt.set()
                if pend.ack is None:    # done sin ack: el PLC sí lo recibió
                    pend.ack = (item[0], 'ok', 'implícito por done')
                    pend.ack_evt.set()

    # -------------------------------------------------------------- estado
    def publish_status(self):
        with self.lock:
            key = (self.connected, self.plc_state, self.detail)
        now = time.monotonic()
        if self.last_status and self.last_status[0] == key and now - self.last_status[1] < 1.0:
            return
        self.last_status = (key, now)
        m = CellStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.cell_id = self.cell_id
        m.broker_connected, m.state, m.detail = key
        m.plc_online = key[1] not in ('offline', 'unknown')
        self.status_pub.publish(m)

    # -------------------------------------------------------------- acción
    def on_goal(self, goal):
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def wait(self, evt, timeout, gh):
        """'ok' | 'timeout' | 'cancel'"""
        end = time.monotonic() + max(0.0, timeout)
        while True:
            if evt.wait(min(0.05, max(0.0, end - time.monotonic()))):
                return 'ok'
            if gh.is_cancel_requested:
                return 'cancel'
            if time.monotonic() >= end:
                return 'timeout'

    def execute(self, gh):
        try:
            return self._execute(gh)
        finally:
            with self.lock:
                self.pending = None
                self.busy = False

    def _execute(self, gh):
        g = gh.request
        res = R()
        fb = NotifyCell.Feedback()
        t0 = time.monotonic()
        total = g.timeout_s if g.timeout_s > 0 else self.done_timeout

        def end(code, msg):
            res.success = code == R.OK
            res.error_code = code
            res.message = msg
            if code == R.CANCELED:
                gh.canceled()
            elif res.success:
                gh.succeed()
            else:
                gh.abort()
            log = self.get_logger().info if res.success else self.get_logger().warning
            log(f'notify_placed seq={res.seq}: {msg}')
            return res

        # 1) PLC listo
        fb.stage = 'waiting_ready'
        gh.publish_feedback(fb)
        deadline = t0 + self.ready_timeout
        while True:
            with self.lock:
                conn, state = self.connected, self.plc_state
            if conn and (state == 'ready' or not self.require_ready):
                break
            if gh.is_cancel_requested:
                return end(R.CANCELED, 'cancelado')
            if time.monotonic() >= deadline:
                if not conn:
                    return end(R.BROKER_DOWN, 'sin conexión con el broker de la celda')
                return end(R.NOT_READY, f'el PLC no está listo (estado {state})')
            time.sleep(0.05)

        # 2) placed con reintentos (mismo seq)
        with self.lock:
            self.seq = p.next_seq(self.seq)
            pend = Pending(self.seq)
            self.pending = pend
        res.seq = pend.seq
        payload = p.encode_placed(self.robot_id, pend.seq, g.piece)
        for attempt in range(1, self.ack_retries + 2):
            fb.stage, fb.attempt = 'sent', attempt
            gh.publish_feedback(fb)
            self.client.publish(self.topics['placed'], payload, qos=1, retain=False)
            r = self.wait(pend.ack_evt, self.ack_timeout, gh)
            if r == 'ok':
                break
            if r == 'cancel':
                return end(R.CANCELED, 'cancelado esperando el ack')
        else:
            return end(R.ACK_TIMEOUT, f'sin ack tras {self.ack_retries + 1} envíos')

        _, result, msg = pend.ack
        if result != 'ok':
            return end(R.REJECTED, f'el PLC rechazó la pieza: {msg or result}')
        fb.stage = 'acked'
        gh.publish_feedback(fb)
        if not g.wait_done:
            return end(R.OK, 'ack recibido')

        # 3) done
        r = self.wait(pend.done_evt, total - (time.monotonic() - t0), gh)
        if r == 'cancel':
            return end(R.CANCELED, 'cancelado esperando done')
        if r == 'timeout':
            return end(R.DONE_TIMEOUT, 'el PLC no reportó done a tiempo')
        _, result, msg = pend.done
        if result != 'ok':
            return end(R.DONE_ERROR, f'el PLC reportó error: {msg or result}')
        fb.stage = 'done'
        gh.publish_feedback(fb)
        return end(R.OK, 'ack y done recibidos')

    def close(self):
        try:
            info = self.client.publish(self.topics['robot_status'],
                                       p.encode_robot_status(self.robot_id, 'offline'),
                                       qos=1, retain=True)
            info.wait_for_publish(timeout=1.0)
        except Exception:  # noqa: BLE001  (cierre best-effort)
            pass
        self.client.disconnect()
        self.client.loop_stop()


def main(args=None):
    rclpy.init(args=args)
    node = CellClient()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
