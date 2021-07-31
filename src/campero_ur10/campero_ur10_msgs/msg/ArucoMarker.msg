########################################
# CONSTANTS
########################################
# longitud vector de esquinas
uint8 NUM_IMG_POINTS = 4

########################################
# Message
########################################
# identificador
uint32 id
# posición y orientacion
geometry_msgs/Pose pose
# posicion de las esquinas en la imagen
geometry_msgs/Point32[] img_points
