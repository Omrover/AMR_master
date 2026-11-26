import json

import pytest

from amr_cell import protocol as p


def test_topics():
    t = p.topics('omrover/cell/st1/')
    assert t['placed'] == 'omrover/cell/st1/robot/placed'
    assert t['plc_status'] == 'omrover/cell/st1/plc/status'


def test_seq():
    assert p.initial_seq(1764000123.9) == 1764000123
    assert p.next_seq(p.UINT32_MAX) == 0


def test_placed_roundtrip():
    s = p.encode_placed('mm01', 42, 'pieza_a', t_ms=5)
    assert json.loads(s) == {'v': 1, 'robot': 'mm01', 'seq': 42, 'piece': 'pieza_a', 't_ms': 5}
    assert p.decode_placed(s.encode()) == ('mm01', 42, 'pieza_a')
    with pytest.raises(p.ProtocolError):
        p.encode_placed('mm01', -1, 'x')


def test_ack_done():
    assert p.decode_ack(p.encode_ack(7, 'ok')) == (7, 'ok', '')
    assert p.decode_ack('{"v":1,"seq":7,"result":"rejected","msg":"ocupada"}') == \
        (7, 'rejected', 'ocupada')
    assert p.decode_done(p.encode_done(7, 'error', 'x')) == (7, 'error', 'x')
    assert p.decode_ack('{"v":1,"seq":7.0,"result":"ok","extra":1}')[0] == 7


@pytest.mark.parametrize('payload', [
    b'\xff', 'no json', '[1]', '{"seq":1,"result":"ok"}', '{"v":2,"seq":1,"result":"ok"}',
    '{"v":1,"seq":-1,"result":"ok"}', '{"v":1,"seq":1.5,"result":"ok"}',
    '{"v":1,"seq":true,"result":"ok"}', '{"v":1,"seq":4294967296,"result":"ok"}',
    '{"v":1,"seq":1,"result":"done"}', '{"v":1,"result":"ok"}',
])
def test_ack_invalid(payload):
    with pytest.raises(p.ProtocolError):
        p.decode_ack(payload)


def test_done_rejects_ack_result():
    with pytest.raises(p.ProtocolError):
        p.decode_done(p.encode_ack(1, 'rejected'))


def test_plc_status():
    assert p.decode_plc_status(p.encode_plc_status('busy')) == 'busy'
    assert p.decode_plc_status(b'') == 'offline'
    with pytest.raises(p.ProtocolError):
        p.decode_plc_status('{"v":1,"state":"idle"}')
