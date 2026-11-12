"""MCU simulado sobre Gazebo: el lado Tlaloc del contrato MQTT v1 con física de gz.

En simulación corre la misma pila que en el robot:

    Gazebo <-ros_gz_bridge-> gz_mcu <-MQTT-> amr_mqtt_bridge -> /odom, /imu/data, ...

Reutiliza EmulatedMcu (watchdog de cmd/vel, e-stop, deduplicación por seq,
acks y perfiles trapezoidales sincronizados del brazo). La base se mueve con
VelocityControl, la "odometría del EKF" se integra con la velocidad medida
por OdometryPublisher, la IMU viene del sensor gz y el brazo y gripper siguen
las consignas con JointPositionController. El tiempo del contrato (t_ms) es
el tiempo de simulación.
"""
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float64

from amr_mqtt_bridge import protocol as p
from amr_mqtt_bridge.arm_units import ArmUnits
from amr_mqtt_bridge.mcu_emulator import EmulatedMcu, MqttMcuRunner
from amr_mqtt_bridge.omni import integrate_pose

ARM_JOINTS = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']
FINGER_JOINTS = ['gripper_finger_joint', 'gripper_finger_right_joint']
FULL_VOLT = 25.2
SEND_PERIOD = 0.02        # s de simulación entre consignas a gz (50 Hz)


class GzMcu(EmulatedMcu):
    def __init__(self, sim_now, units, finger_travel, send_cmd_vel, send_joint,
                 dock=None, recharge_v_per_s=0.05, **kw):
        super().__init__(grip_object_at=None, clock=sim_now, **kw)
        self.units = units
        self.travel = finger_travel
        self.send_cmd_vel = send_cmd_vel
        self.send_joint = send_joint
        self.dock = dock                       # (x, y, radio) o None
        self.recharge = recharge_v_per_s
        self.last_sim = None
        self.meas_twist = (0.0, 0.0, 0.0)
        self.gz_pose = (0.0, 0.0)
        self.imu = None                        # (q[w,x,y,z], gyro, acc)
        self.joint_pos = {}
        self.joint_vel = {}
        self.arm_ready = False
        self.finger_last = None
        self.finger_still_since = None
        self.last_send = -1.0
        self.sent_joint = {}
        self.last_grip_target = None

    # El contrato usa el tiempo de simulación.
    def now_ms(self):
        return int(self.clock() * 1000.0)

    # ------------------------------------------------------- entradas de gz
    def on_odom(self, msg):
        t = msg.twist.twist
        self.meas_twist = (t.linear.x, t.linear.y, t.angular.z)
        self.gz_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_imu(self, msg):
        o, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        self.imu = ([o.w, o.x, o.y, o.z], [g.x, g.y, g.z], [a.x, a.y, a.z])

    def on_joints(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_pos[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_vel[name] = msg.velocity[i]

    def override_battery(self, volt):
        self.battery = max(0.0, min(30.0, float(volt)))

    # ----------------------------------------------------------------- física
    def step(self, _wall_dt):
        sim = self.clock()
        if self.last_sim is None:
            self.last_sim = sim
            return
        dt = sim - self.last_sim
        if dt <= 0.0:
            return
        self.last_sim = sim
        now = self.now_ms()

        # Watchdog y rampa de la consigna (igual que el firmware).
        if self.cmd_active and now >= self.cmd_until_ms:
            self.cmd_active = False
            self.wd_tripped = True
        target = self.cmd if (self.cmd_active and not self.estop) else (0.0, 0.0, 0.0)
        self.vx = _approach(self.vx, target[0], self.lin_acc * dt)
        self.vy = _approach(self.vy, target[1], self.lin_acc * dt)
        self.wz = _approach(self.wz, target[2], self.ang_acc * dt)
        # Consignas a gz a 50 Hz (no en cada paso): limita la carga de ROS/bridge.
        send = sim - self.last_send >= SEND_PERIOD
        if send:
            self.last_send = sim
            self.send_cmd_vel(self.vx, self.vy, self.wz)

        # "EKF": integra la velocidad medida (permite odom_reset como el MCU real).
        mvx, mvy, mwz = self.meas_twist
        self.x, self.y, self.yaw = integrate_pose(self.x, self.y, self.yaw, mvx, mvy, mwz, dt)
        self.wheel_vel = self.kin.inverse(mvx, mvy, mwz)
        self.wheel_pos = [q + w * dt for q, w in zip(self.wheel_pos, self.wheel_vel)]

        self._step_arm(dt, send)
        self._step_gripper(sim, send)

        moving = abs(mvx) + abs(mvy) + abs(mwz) > 1e-3
        self.battery = max(18.0, self.battery - self.drain * dt * (3.0 if moving else 1.0))
        if self.dock is not None and not moving and \
                math.hypot(self.gz_pose[0] - self.dock[0], self.gz_pose[1] - self.dock[1]) < self.dock[2]:
            self.battery = min(FULL_VOLT, self.battery + self.recharge * dt)

    def _send_joint(self, name, value):
        """Envía una consigna solo si cambió (las de reposo no se repiten)."""
        if abs(self.sent_joint.get(name, math.inf) - value) > 1e-5:
            self.sent_joint[name] = value
            self.send_joint(name, value)

    def _step_arm(self, dt, send=True):
        if not self.arm_ready:
            if not all(n in self.joint_pos for n in ARM_JOINTS):
                return
            counts = self.units.rad_to_counts([self.joint_pos[n] for n in ARM_JOINTS])
            for j, c in zip(self.joints, counts):
                j.pos = j.target = float(c)
                j.vel = 0.0
            self.arm_ready = True
        if self.arm_enabled and not self.estop:
            for j in self.joints:
                j.step(dt)
        if not send:
            return
        rad = self.units.counts_to_rad([j.pos for j in self.joints])
        for name, q in zip(ARM_JOINTS, rad):
            self._send_joint(name, q)

    def _step_gripper(self, sim, send=True):
        finger = self.joint_pos.get(FINGER_JOINTS[0])
        if finger is None:
            return
        self.grip_pos = min(1.0, max(0.0, 1.0 - finger / self.travel))
        commanded = (1.0 - self.grip_target) * self.travel
        # El tiempo "quieto" cuenta desde el último movimiento del dedo O desde la
        # última consigna: un dedo que estaba en reposo antes del comando de cierre
        # no debe interpretarse como atascado contra una pieza.
        if (self.finger_last is None or abs(finger - self.finger_last) > 2e-4 or
                self.grip_target != self.last_grip_target):
            self.finger_last = finger
            self.finger_still_since = sim
            self.last_grip_target = self.grip_target
        closing = commanded < finger - 0.002
        if self.grasped:
            commanded = max(0.0, finger - 0.001)      # mantiene el apriete
            self.grip_cur = self.grip_max_cur * 0.9
        elif closing and sim - self.finger_still_since > 0.3:
            self.grasped = True                       # el dedo se detuvo antes de llegar
            self.grip_cur = self.grip_max_cur + 0.05
        else:
            self.grip_cur = 0.15 if abs(commanded - finger) > 0.002 else 0.02
        if self.estop:
            commanded = finger
        if send:
            for name in FINGER_JOINTS:
                self._send_joint(name, commanded)

    # ------------------------------------------------------------- telemetría
    def telemetry(self, topic):
        t = self.now_ms()
        if topic == 'llc/odom':
            mvx, mvy, mwz = self.meas_twist
            return p.encode_telemetry(topic, self.boot_id, t, x=self.x, y=self.y, yaw=self.yaw,
                                      vx=mvx, vy=mvy, wz=mwz,
                                      cov=[1e-3, 1e-3, 2e-3, 4e-4, 4e-4, 1e-3])
        if topic == 'llc/imu' and self.imu is not None:
            q, g, a = self.imu
            norm = math.sqrt(sum(v * v for v in q)) or 1.0
            return p.encode_telemetry(topic, self.boot_id, t, q=[v / norm for v in q],
                                      gyro=g, acc=a, acc_status=3)
        if topic == 'llc/arm' and self.arm_ready:
            meas = [self.joint_pos[n] for n in ARM_JOINTS]
            counts = self.units.rad_to_counts(meas)
            k = 60.0 / (2.0 * math.pi) * self.units.gear_ratio
            vel = [s * self.joint_vel.get(n, 0.0) * k for n, s in zip(ARM_JOINTS, self.units.sign)]
            in_pos = [abs(j.target - c) < 25 and abs(j.vel) < 1.0 for j, c in zip(self.joints, counts)]
            return p.encode_telemetry(
                topic, self.boot_id, t, enabled=self.arm_enabled, pos=counts, vel=vel,
                cur=[0.1 + abs(v) * 0.01 for v in vel], temp=[32.0] * 4, in_pos=in_pos,
                fault=[0] * 4, grip={'pos': self.grip_pos, 'cur': self.grip_cur,
                                     'grasped': self.grasped})
        return super().telemetry(topic)


def _approach(value, target, step):
    if value < target:
        return min(value + step, target)
    return max(value - step, target)


class GzMcuNode(Node):
    def __init__(self):
        super().__init__('gz_mcu')
        d = self.declare_parameter
        d('mqtt_host', '127.0.0.1')
        d('mqtt_port', 18883)
        d('mqtt_prefix', 'omrover/mm01')
        d('wheel_radius', 0.0762)
        d('half_length', 0.1583)    # ruedas mecanum en (+-half_length, +-half_width)
        d('half_width', 0.1734)
        d('battery_volt', 25.0)
        d('drain_v_per_s', 0.002)
        d('recharge_v_per_s', 0.05)
        d('dock_xyr', [-1.6, 0.0, 0.35])   # dock dummy del mundo warehouse (x, y, radio)
        d('arm_counts_per_rev', 16384)
        d('arm_gear_ratio', 1.0)
        d('arm_zero_offset', [0, 0, 0, 0])
        d('arm_sign', [1, 1, 1, 1])
        d('arm_lower', [-math.pi, -math.pi / 2, -2.5, -2.5])
        d('arm_upper', [math.pi, math.pi / 2, 2.5, 2.5])
        d('gripper_travel', 0.035)
        # Parámetros de arm.yaml que este nodo no usa (se cargan con /**).
        for name, value in (('arm_joints', ARM_JOINTS), ('arm_default_accel', 2.0),
                            ('arm_status_rate_hz', 10.0), ('gripper_joint', FINGER_JOINTS[0])):
            d(name, value)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        units = ArmUnits(int(g('arm_counts_per_rev')), float(g('arm_gear_ratio')),
                         list(g('arm_zero_offset')), list(g('arm_sign')),
                         list(g('arm_lower')), list(g('arm_upper')))
        self.cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.joint_pubs = {n: self.create_publisher(Float64, f'/sim/{n}/cmd_pos', 10)
                           for n in ARM_JOINTS + FINGER_JOINTS}
        dock = list(g('dock_xyr'))
        self.mcu = GzMcu(
            sim_now=lambda: self.get_clock().now().nanoseconds * 1e-9,
            units=units, finger_travel=float(g('gripper_travel')),
            send_cmd_vel=self.send_cmd_vel, send_joint=self.send_joint,
            dock=tuple(dock) if len(dock) == 3 else None,
            recharge_v_per_s=float(g('recharge_v_per_s')),
            wheel_radius=float(g('wheel_radius')), half_length=float(g('half_length')),
            half_width=float(g('half_width')),
            battery_volt=float(g('battery_volt')), drain_v_per_s=float(g('drain_v_per_s')))
        self.create_subscription(Odometry, '/sim/odom', self.mcu.on_odom, 10)
        self.create_subscription(Imu, '/sim/imu', self.mcu.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/sim/joint_states', self.mcu.on_joints, 10)
        self.create_subscription(Float32, '/sim/battery_override',
                                 lambda m: self.mcu.override_battery(m.data), 10)
        self.mqtt = (g('mqtt_host'), int(g('mqtt_port')), g('mqtt_prefix'))

    def send_cmd_vel(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(msg)

    def send_joint(self, name, value):
        self.joint_pubs[name].publish(Float64(data=float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = GzMcuNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    # Espera el primer /clock (con use_sim_time el reloj arranca en 0).
    while rclpy.ok() and node.get_clock().now().nanoseconds == 0:
        time.sleep(0.05)
    host, port, prefix = node.mqtt
    runner = MqttMcuRunner(node.mcu, host=host, port=port, prefix=prefix)
    node.get_logger().info(f'MCU simulado boot_id={node.mcu.boot_id} -> {host}:{port} {prefix}')
    try:
        runner.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
