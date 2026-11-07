# Estado del controlador de bajo nivel (Tlaloc S32K344) visto por el bridge MQTT.
std_msgs/Header header

# Estado reportado por el MCU: "init", "ok", "estop", "fault" u "offline" (LWT).
string state
string boot_id

# Salud del enlace calculada en la NUC.
bool mqtt_connected
bool link_live            # telemetría recibida hace menos de stale_timeout
float32 link_age_s        # segundos desde la última telemetría (-1 si nunca)
uint32 invalid_packets    # payloads rechazados por el validador del contrato

# Campos del MCU.
bool estop
bool watchdog_tripped     # el MCU detuvo la base por comando expirado
uint32 faults             # bitmask, ver AMR_LLC/protocol/CONTRACT.md
float32 cpu_load          # 0..1
int32[] roboclaw_errors   # un valor por RoboClaw (dir 128, 129)
