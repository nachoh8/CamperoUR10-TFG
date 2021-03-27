# campero_ur10_ws
Prácticas en el i3A y TFG:
Uso del entorno ROS con la interfaz Rviz para el control del robot manipulador UR10, con el objetivo de que el robot pueda realizar gráficos en papel

## Requisitos
- Ubuntu 16.04
- ROS Kinetic
- MoveIt
- OpenCV
- Python 2.7
- C++ 11

## Compilar
```bash
catkin_make
```

## Paquetes
- **campero_ur10_moveit**
    - Descripción: Paquete necesario para la simulación y movimiento del robot con MoveIt
    - Uso:
    ```bash
        roslaunch campero_ur10_moveit demo.launch [scene_file:=scene.scene]
    ```
     por defecto scene_file:=ws_walls.scene

- **campero_ur10_msgs**
    - Descripción: Paquete que contiene la definición de mensajes para operar el robot
    - Uso: ~

- **campero_ur10_server**
    - Descripción: Paquete que planifica y ejecuta las trayectorias del robot, hace de intermediario entre las aplicaciones a nivel de usuario y el robot en simulación/real
    - Uso:
    ```bash
        roslaunch campero_ur10_server campero_ur10_server.launch mode:=(teleop | draw)
    ```
    
- **campero_ur10_op**
    - Descripción: Paquete para teleoperar el robot mediante teclado, tiene dos modos(cartesiano y articular),envía los conmandos al nodo campero_ur10_server
    - Uso:
    ```bash
        python campero_ur10_op.py [-ms <max_step>] [-t <time>]
    ```
     por defecto -ms = 0.1 y -t = 1
     
- **description**
    - Descripción: Paquete que contiene archivos urdf/xacro que definen el robot
    - Uso: ~
 
 - **draw_board**
    - Descripción: Paquete que lanza una pizarra para dibujar, la imagen realizada se envía al nodo campero_ur10_server
    - Uso:
    ```bash
        python draw_board_node.py [-s <size>]
    ```
     por defecto -s = 512 pixeles, crea una pizarra de SxS pixeles
