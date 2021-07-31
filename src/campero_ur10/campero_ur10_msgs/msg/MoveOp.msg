########################################
# CONSTANTS
########################################
# tipo: coordenadas articulares
uint8 MOVE_JOINT = 0
# tipo: coordenadas cartesianas
uint8 MOVE_CARTHESIAN = 1
# IDs Articulaciones
uint8 C_UR10_SHOULDER_PAN_JOINT_IDX = 0
uint8 C_UR10_SHOULDER_LIFT_JOINT_IDX = 1
uint8 C_UR10_ELBOW_JOINT_IDX = 2
uint8 C_UR10_W1_JOINT_IDX = 3
uint8 C_UR10_W2_JOINT_IDX = 4
uint8 C_UR10_W3_JOINT_IDX = 5
# IDs Cartesianas
# posición
uint8 X_AXIS = 0
uint8 Y_AXIS = 1
uint8 Z_AXIS = 2
uint8 ALL_CARTH_AXIS = 3
# rotación
uint8 RX_AXIS = 4
uint8 RY_AXIS = 5
uint8 RZ_AXIS = 6
########################################
# Message
########################################
uint8 type # tipo movimiento
uint8 id
float32 vx
float32 vy
float32 vz
