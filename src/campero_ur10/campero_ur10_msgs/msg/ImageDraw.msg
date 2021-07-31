#######################################
# CONSTANTS
########################################
# tipo: dibujo real, plano 3D
uint8 REAL_POINTS = 0
# tipo: dibujo virtual, plano 2D
uint8 BOARD_POINTS = 1

########################################
# Message
########################################
uint8 type # tipo de dibujo
ImgTrace[] traces # trazos
int32 W # ancho dibujo virtual
int32 H # altura dibujo virtual
