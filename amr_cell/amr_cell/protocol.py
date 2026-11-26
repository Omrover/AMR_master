"""Codificación y validación del contrato robot <-> celda (CELL_CONTRACT.md).

Python puro (sin paho ni rclpy) para poder probarlo en cualquier máquina.
"""
import json
import math
import time

VERSION = 1
UINT32_MAX = 0xFFFFFFFF
PLC_STATES = ('ready', 'busy', 'fault', 'offline')
ACK_RESULTS = ('ok', 'rejected')
DONE_RESULTS = ('ok', 'error')


class ProtocolError(ValueError):
    pass


def topics(prefix):
    """Topics del contrato a partir de omrover/cell/<cell_id>."""
    p = prefix.rstrip('/')
    return {
        'placed': f'{p}/robot/placed',
        'robot_status': f'{p}/robot/status',
        'ack': f'{p}/plc/ack',
        'done': f'{p}/plc/done',
        'plc_status': f'{p}/plc/status',
    }


def initial_seq(now=None):
    """Segundos Unix: no se repiten seq al reiniciar el robot."""
    return int(time.time() if now is None else now) & UINT32_MAX


def next_seq(seq):
    return (seq + 1) & UINT32_MAX


def _dumps(obj):
    return json.dumps(obj, separators=(',', ':'), ensure_ascii=False)


def encode_placed(robot, seq, piece, t_ms=None):
    if not 0 <= int(seq) <= UINT32_MAX:
        raise ProtocolError('seq fuera de rango')
    return _dumps({'v': VERSION, 'robot': robot, 'seq': int(seq), 'piece': str(piece),
                   't_ms': int(time.time() * 1000) if t_ms is None else int(t_ms)})


def encode_robot_status(robot, state):
    return _dumps({'v': VERSION, 'robot': robot, 'state': state})


def encode_ack(seq, result, msg=''):
    d = {'v': VERSION, 'seq': int(seq), 'result': result}
    if msg:
        d['msg'] = msg
    return _dumps(d)


encode_done = encode_ack


def encode_plc_status(state):
    return _dumps({'v': VERSION, 'state': state})


def _load(payload):
    if isinstance(payload, (bytes, bytearray)):
        try:
            payload = payload.decode('utf-8')
        except UnicodeDecodeError as e:
            raise ProtocolError('no es UTF-8') from e
    try:
        d = json.loads(payload)
    except (ValueError, TypeError) as e:
        raise ProtocolError('JSON inválido') from e
    if not isinstance(d, dict):
        raise ProtocolError('se esperaba un objeto JSON')
    if d.get('v') != VERSION:
        raise ProtocolError(f'versión {d.get("v")!r} no soportada')
    return d


def _seq(d):
    s = d.get('seq')
    if isinstance(s, bool) or not isinstance(s, (int, float)) or not math.isfinite(s) \
            or s != int(s) or not 0 <= s <= UINT32_MAX:
        raise ProtocolError('seq inválido')
    return int(s)


def _result(d, allowed):
    r = d.get('result')
    if r not in allowed:
        raise ProtocolError(f'result {r!r} inválido')
    msg = d.get('msg', '')
    return r, msg if isinstance(msg, str) else str(msg)


def decode_ack(payload):
    """-> (seq, result, msg)"""
    d = _load(payload)
    return (_seq(d),) + _result(d, ACK_RESULTS)


def decode_done(payload):
    """-> (seq, result, msg)"""
    d = _load(payload)
    return (_seq(d),) + _result(d, DONE_RESULTS)


def decode_plc_status(payload):
    """-> state. Un payload vacío (retenido borrado) cuenta como offline."""
    if payload in (b'', ''):
        return 'offline'
    d = _load(payload)
    s = d.get('state')
    if s not in PLC_STATES:
        raise ProtocolError(f'state {s!r} inválido')
    return s


def decode_placed(payload):
    """-> (robot, seq, piece). Lo usa el PLC simulado."""
    d = _load(payload)
    return str(d.get('robot', '')), _seq(d), str(d.get('piece', ''))
