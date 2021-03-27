# campero_ur10_ws
Prácticas en el i3A y TFG
Uso del entorno ROS con la interfaz Rviz para el control del robot manipulador UR10, con el objetivo de que el robot pueda realizar gráficos en papel

## Paquetes
- **campero_ur_moveit**
    - Descripción: Paquete necesario para la planificación y ejecución de trayctorias con MoveIt
    - Uso: ```bash
    roslaunch campero_ur10_moveit demo.launch [scene_file:=scene.scene]
    ```
    por defecto scene_file:=ws_walls.scene