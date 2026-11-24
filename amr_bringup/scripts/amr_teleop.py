#!/usr/bin/env python3
"""Teleop de teclado para la base holonómica, estilo turtlebot3_teleop.

Cada tecla SUBE o BAJA la velocidad un paso y el nodo la sigue publicando en
/cmd_vel a rate_hz aunque no se toque nada (no hace falta mantener la tecla).

    ros2 run amr_bringup amr_teleop.py [--ros-args -p lin_step:=0.02 ...]

    w / x  (↑ / ↓)   vx  + / -     adelante / atrás
    a / d  (← / →)   vy  + / -     izquierda / derecha (lateral)
    q / e            wz  + / -     girar izquierda / derecha
    s  o  espacio    ALTO (todo a cero)
    Ctrl-C           alto y salir

Al salir (o si el nodo muere) deja de publicar y el bridge manda cero tras
cmd_vel_timeout (0.2 s); además, al cerrar publica cero explícitamente.
"""
import os
import select
import sys

HELP = __doc__.split('\n\n')[2]

# tecla -> (eje, signo); eje 0 = vx, 1 = vy, 2 = wz
KEYS = {
    'w': (0, 1), 'x': (0, -1), 'a': (1, 1), 'd': (1, -1), 'q': (2, 1), 'e': (2, -1),
    'W': (0, 1), 'X': (0, -1), 'A': (1, 1), 'D': (1, -1), 'Q': (2, 1), 'E': (2, -1),
    '\x1b[A': (0, 1), '\x1b[B': (0, -1), '\x1b[D': (1, 1), '\x1b[C': (1, -1),
}
STOP_KEYS = ('s', 'S', ' ')


def apply_key(vel, key, steps, limits):
    """Nueva velocidad [vx, vy, wz] tras una tecla. Devuelve None si la tecla no aplica."""
    if key in STOP_KEYS:
        return [0.0, 0.0, 0.0]
    if key not in KEYS:
        return None
    axis, sign = KEYS[key]
    out = list(vel)
    v = round(out[axis] + sign * steps[axis], 4)
    out[axis] = max(-limits[axis], min(limits[axis], v))
    return out


def read_key(fd, timeout):
    """Lee una tecla (o una secuencia de flecha ESC [ X). '' si no hubo."""
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return ''
    ch = os.read(fd, 1).decode(errors='ignore')
    if ch == '\x1b':
        r, _, _ = select.select([fd], [], [], 0.01)
        if r:
            ch += os.read(fd, 2).decode(errors='ignore')
    return ch


def main():
    import termios
    import tty

    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped

    if not sys.stdin.isatty():
        print('amr_teleop necesita una terminal interactiva (no ros2 launch).')
        return 1

    rclpy.init()
    node = rclpy.create_node('amr_teleop')
    p = node.declare_parameter
    rate_hz = p('rate_hz', 20.0).value
    steps = (p('lin_step', 0.02).value, p('lat_step', 0.02).value, p('ang_step', 0.1).value)
    limits = (p('max_vx', 0.35).value, p('max_vy', 0.20).value, p('max_wz', 0.65).value)
    stamped = p('stamped', False).value
    pub = node.create_publisher(TwistStamped if stamped else Twist, 'cmd_vel', 10)

    def publish(v):
        msg = TwistStamped() if stamped else Twist()
        t = msg.twist if stamped else msg
        if stamped:
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'
        t.linear.x, t.linear.y, t.angular.z = float(v[0]), float(v[1]), float(v[2])
        pub.publish(msg)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    vel = [0.0, 0.0, 0.0]
    print(HELP)
    print(f'pasos: vx/vy {steps[0]:.2f}/{steps[1]:.2f} m/s, wz {steps[2]:.2f} rad/s; '
          f'límites {limits[0]:.2f} / {limits[1]:.2f} m/s, {limits[2]:.2f} rad/s\n')
    try:
        tty.setcbreak(fd)
        period = 1.0 / rate_hz
        while rclpy.ok():
            key = read_key(fd, period)
            if key == '\x03':
                break
            new = apply_key(vel, key, steps, limits) if key else None
            if new is not None and new != vel:
                vel = new
                sys.stdout.write(f'\rvx {vel[0]:+.2f} m/s   vy {vel[1]:+.2f} m/s   '
                                 f'wz {vel[2]:+.2f} rad/s     ')
                sys.stdout.flush()
            publish(vel)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        for _ in range(3):
            publish((0.0, 0.0, 0.0))
        print('\nalto')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
