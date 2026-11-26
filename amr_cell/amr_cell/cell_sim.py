"""PLC de la celda simulado (solo paho, sin ROS) para probar el contrato y la misión.

    ros2 run amr_cell cell_sim --port 18883 --prefix omrover/cell/st1
    python3 -m amr_cell.cell_sim --ack-delay 0.3 --done-delay 3 [--reject] [--drop-acks 1]

Publica plc/status retenido (ready/busy) con LWT offline, responde robot/placed
con ack (deduplicando por seq) y, si no se pasa --no-done, con done tras
--done-delay segundos (busy mientras tanto).
"""
import argparse
import signal
import sys
import threading
import time

from amr_cell import protocol as p
from amr_cell.mqtt_support import make_client


class CellSim:
    def __init__(self, a):
        self.a = a
        self.t = p.topics(a.prefix)
        self.lock = threading.Lock()
        self.seen = {}                  # seq -> ack payload ya enviado
        self.drop = a.drop_acks
        self.client = make_client(client_id=f'cell-sim-{int(time.time())}',
                                  will=(self.t['plc_status'], p.encode_plc_status('offline'),
                                        1, True))
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def log(self, msg):
        print(f'[cell_sim] {msg}', flush=True)

    def status(self, state):
        self.client.publish(self.t['plc_status'], p.encode_plc_status(state), qos=1, retain=True)
        self.log(f'status {state}')

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(self.t['placed'], qos=1)
        self.status(self.a.state)
        self.log('listo')

    def on_message(self, client, userdata, msg):
        try:
            robot, seq, piece = p.decode_placed(msg.payload)
        except p.ProtocolError as e:
            self.log(f'placed inválido: {e}')
            return
        with self.lock:
            if seq in self.seen:        # reintento del robot: repetir el ack, no mover
                self.log(f'seq {seq} repetido: reenvío ack')
                client.publish(self.t['ack'], self.seen[seq], qos=1)
                return
            if self.drop > 0:
                self.drop -= 1
                self.log(f'placed seq {seq} ({piece}) de {robot}: ack descartado (prueba)')
                return
            result = 'rejected' if self.a.reject else 'ok'
            ack = p.encode_ack(seq, result, 'rechazo de prueba' if self.a.reject else '')
            self.seen[seq] = ack
        self.log(f'placed seq {seq} ({piece}) de {robot}: {result}')

        def respond():
            client.publish(self.t['ack'], ack, qos=1)
            if result != 'ok' or self.a.no_done:
                return
            self.status('busy')
            time.sleep(self.a.done_delay)
            client.publish(self.t['done'], p.encode_done(seq, 'error' if self.a.done_error
                                                         else 'ok'), qos=1)
            self.log(f'done seq {seq}')
            self.status('ready')

        threading.Timer(self.a.ack_delay, respond).start()

    def run(self):
        self.client.connect(self.a.host, self.a.port, keepalive=10)
        self.client.loop_forever()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=18883)
    ap.add_argument('--prefix', default='omrover/cell/st1')
    ap.add_argument('--state', default='ready', choices=['ready', 'busy', 'fault'])
    ap.add_argument('--ack-delay', type=float, default=0.3)
    ap.add_argument('--done-delay', type=float, default=3.0)
    ap.add_argument('--reject', action='store_true')
    ap.add_argument('--done-error', action='store_true')
    ap.add_argument('--no-done', action='store_true')
    ap.add_argument('--drop-acks', type=int, default=0, help='ignora los primeros N placed')
    # ros2 run / launch agregan --ros-args ...: se ignoran
    argv = sys.argv[1:] if argv is None else argv
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    sim = CellSim(ap.parse_args(argv))

    def stop(*_):
        # salida limpia: el LWT no se dispara, así que offline a mano; loop_forever
        # envía ambos paquetes en orden y regresa tras el DISCONNECT
        sim.status('offline')
        sim.client.disconnect()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    sim.run()


if __name__ == '__main__':
    main()
