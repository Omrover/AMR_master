# Estado del brazo de 4 GIM6010-8 y del gripper (motor DC con DRV8833 + ACS712).
std_msgs/Header header

bool enabled
bool[] in_position          # por junta, reportado por el MCU
uint32[] motor_faults       # código de falla 0xAE por junta (0 = sin falla)
float32[] temperature_c
float32[] current_a

float32 gripper_position    # 0 = abierto, 1 = cerrado (estimada por tiempo)
float32 gripper_current_a   # medido con el ACS712
bool gripper_grasped        # se trabó sobre una pieza y la sostiene con max_current_a
