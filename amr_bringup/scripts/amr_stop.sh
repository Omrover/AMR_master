#!/usr/bin/env bash
# Detiene la pila de simulación/emulación del Omrover.
for pat in 'gz sim' 'gz_mcu' 'parameter_bridge' 'amr_mqtt_bridge/bridge' 'mcu_emulator' 'amr_local.conf' 'robot_state_publisher' 'ros2 launch amr_'; do
  pgrep -f -- "$pat" | grep -vx "$$" | xargs -r kill -INT 2>/dev/null
done
sleep 3
for pat in 'gz sim' 'gz_mcu' 'parameter_bridge' 'amr_mqtt_bridge/bridge' 'mcu_emulator' 'amr_local.conf' 'robot_state_publisher'; do
  pgrep -f -- "$pat" | grep -vx "$$" | xargs -r kill -9 2>/dev/null
done
true
