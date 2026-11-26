# Contrato MQTT robot ↔ celda (PLC) — BORRADOR v1

Estado: **propuesta para revisar con el equipo del PLC**. Las secciones marcadas
como *abierto* están por definir. El lado del robot (`amr_cell`) ya implementa
este borrador y trae un PLC simulado (`cell_sim`) para probar la misión sin la celda.

## Flujo

```
robot                                   PLC
  │  (lee plc/status = ready, retenido)  │
  │── robot/placed {seq, piece} ────────►│   el robot ya soltó la pieza y se retiró
  │◄──────────────── plc/ack {seq, ok} ──│   recibido: el PLC va a mover su brazo
  │◄──────────── plc/status = busy ──────│
  │◄─────────────── plc/done {seq, ok} ──│   (opcional) el brazo del PLC tomó la pieza
  │◄──────────── plc/status = ready ─────│
```

El robot publica `placed` **después** de abrir el gripper y retirar el brazo de
la zona de la celda: el PLC puede mover su brazo en cuanto lo recibe.

## Topics

Prefijo: `omrover/cell/<cell_id>` (por defecto `cell_id = st1`). Todo en JSON UTF-8,
objetos planos (sin anidar) para que sea fácil de armar y leer en el PLC. Campos
desconocidos se ignoran. `v` es la versión del contrato (1).

| Topic | Dir | QoS | Retain | Payload |
|---|---|---|---|---|
| `…/robot/placed` | robot → PLC | 1 | no | `{"v":1,"robot":"mm01","seq":1764000123,"piece":"pieza_a","t_ms":1764000123456}` |
| `…/robot/status` | robot → PLC | 1 | **sí** | `{"v":1,"robot":"mm01","state":"online"}`; LWT `{"v":1,"robot":"mm01","state":"offline"}` |
| `…/plc/ack` | PLC → robot | 1 | no | `{"v":1,"seq":1764000123,"result":"ok"}` o `"result":"rejected","msg":"estación ocupada"` |
| `…/plc/done` | PLC → robot | 1 | no | `{"v":1,"seq":1764000123,"result":"ok"}` o `"result":"error","msg":"..."` |
| `…/plc/status` | PLC → robot | 1 | **sí** | `{"v":1,"state":"ready"}` (`ready` / `busy` / `fault`); LWT `{"v":1,"state":"offline"}` |

## Reglas

- **seq**: entero de 32 bits sin signo. El robot lo inicia con los segundos Unix al
  arrancar y lo incrementa en cada pieza, así no repite números tras reiniciarse.
- **Reintentos**: si no llega `plc/ack` con el mismo `seq` en `ack_timeout` (3 s), el
  robot reenvía el **mismo** `placed` (mismo `seq`) hasta `ack_retries` veces (3).
  El PLC debe **deduplicar por seq**: si recibe un `seq` que ya procesó, reenvía el
  ack y no repite el movimiento.
- **ready**: el robot no publica `placed` si `plc/status` no es `ready`. Espera hasta
  `ready_timeout` (10 s) y si no, falla con `NOT_READY` (la misión decide qué hacer).
- `rejected` en el ack: el PLC no moverá su brazo; el robot no reintenta y reporta
  `REJECTED`.
- `plc/status` y `robot/status` van retenidos con LWT, así cada lado sabe al conectarse
  si el otro está vivo. Ningún otro mensaje va retenido (un `placed` retenido se
  reprocesaría al reconectar).

## Lado ROS (amr_cell)

- Acción `cell/notify_placed` (`amr_interfaces/action/NotifyCell`): `piece`, `wait_done`,
  `timeout_s` → `success`, `error_code`, `message`, `seq`.
- Topic `cell/status` (`amr_interfaces/msg/CellStatus`, transient local).

```bash
ros2 launch amr_cell cell.launch.py broker_host:=<ip> cell_id:=st1          # robot
ros2 launch amr_cell cell.launch.py sim:=true                                # + PLC simulado
ros2 action send_goal /cell/notify_placed amr_interfaces/action/NotifyCell "{piece: pieza_a, wait_done: true}" --feedback
```

## Abierto (definir con el equipo del PLC)

1. **Broker**: ¿lo pone el PLC/la celda o usamos el Mosquitto de la NUC? ¿IP, puerto,
   usuario/contraseña o TLS? ¿En qué red está el PLC (la NUC tiene Wi-Fi y eno1 al Tlaloc)?
2. **Formato**: ¿el PLC puede armar/leer JSON (p. ej. Siemens LMQTT + JSON, Node-RED de
   gateway) o prefiere texto plano o valores sueltos por topic?
3. **Identificadores**: `cell_id`, nombres de pieza y si hay varias posiciones de
   colocación (`slot`).
4. **done**: ¿el PLC avisa cuando terminó? ¿El robot debe esperarlo antes de irse o de
   traer la siguiente pieza?
5. **Tiempos**: ack, done y qué hacer si el PLC no responde (reintentar, abortar,
   avisar a un operador).
6. ¿Hay algo que el PLC deba mandarle al robot además de esto (p. ej. "pieza lista
   para recoger" en otra estación)?
