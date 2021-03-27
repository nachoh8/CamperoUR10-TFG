# campero_ur10_ws
Prácticas en el i3A y TFG
Uso del entorno ROS con la interfaz Rviz para el control del robot manipulador UR10, con el objetivo de que el robot pueda realizar gráficos en papel

## Requisitos
- Ubuntu 16.04
- ROS Kinetic
- MoveIt
- Python 2.7
- C++ 11

## Paquetes
- **campero_ur10_moveit**
    - Descripción: Paquete necesario para la planificación y ejecución de trayctorias con MoveIt
    - Uso:
    ```bash
        roslaunch campero_ur10_moveit demo.launch [scene_file:=scene.scene]
    ```
        por defecto scene_file:=ws_walls.scene

- **campero_ur10_msgs**
    - Descripción: Paquete que contiene la definición de mensajes para operar el robot
    - Uso: -

- **campero_ur10_op**
    - Descripción: Paquete para teleoperar el robot mediante el teclado, tiene dos modos(cartesiano y articular)
    - Uso:
    ```bash
        python campero_ur10_op.py [-ms <max_step>] [-t <time>]
    ```
        por defecto -ms = 0.1 y -t = 1