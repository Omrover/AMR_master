# Estado del brazo de 4 GIM6010-8 y del gripper (servo PWM + ACS712).
std_msgs/Header header

bool enabled
bool[] in_position          # por junta, reportado por el MCU
uint32[] motor_faults       # código de falla 0xAE por junta (0 = sin falla)
float32[] temperature_c
float32[] current_a

float32 gripper_position    # 0 = abierto, 1 = cerrado
float32 gripper_current_a   # medido con el ACS712
bool gripper_grasped        # se alcanzó max_current_a al cerrar
