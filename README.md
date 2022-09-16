# Development of a practical demonstrator in ROS for the UR10 robot manipulator
Internships and Bachelor thesis. Development of applications (painting and teleoperation) for the UR10 robot
manipulator with ROS Kinetic, both in simulation and in real environment. In addition to the use of MoveIt, OpenCV and
the Realsense D435 camera. [Documentation](https://deposita.unizar.es/record/63479). [Results](https://www.youtube.com/watch?v=zaabGxoTJe8).

## Requisitos
- Ubuntu 16.04
- ROS Kinetic
- MoveIt
- OpenCV
- Python 2.7
- C++ 11

## Compilar/Ejecutar
- Compilar comando:
```bash
catkin_make
```
- Antes de ejecutar cualquier .launch, es necesario realizar:
```bash
source devel/setup.bash
```
- Si después de haber realizado los pasos anteriores no se encuentran los paquetes al hacer roslaunch, hacer lo siguiente desde el directorio del workspace:
```bash
rospack find campero_ur10_description
rospack find ur10_moveit_official
rospack find campero_ur10_pen_moveit
rospack find campero_ur10_msgs
rospack find campero_ur10_server
source devel/setup.bash
```

## Paquetes    
- **campero_ur10_pen_moveit**
    - Descripción: Paquete necesario para la simulación y movimiento del robot con pinza y rotulador en MoveIt, no funciona con el robot real
    - Uso:
    ```bash
        roslaunch campero_ur10_pen_moveit demo.launch
    ```
- **ur10_moveit_official**
    - Descripción: Paquete necesario para la simulación y movimiento real del robot con pinza en MoveIt, no incluye el prefijo campero_ur10_
    - Uso:
     - simulacion:
    ```bash
        roslaunch ur10_moveit_official demo.launch
    ```
     - real: requiere haber lanzado previamente el driver(campero_ur10_description/campero_ur10_bringup.launch)
    ```bash
        roslaunch ur10_moveit_official ur10_moveit_planning_execution.launch
        roslaunch ur10_moveit_official moveit_rviz.launch config:=true
    ```
     
- **campero_ur10_msgs**
    - Descripción: Paquete que contiene la definición de mensajes para operar el robot
    - Uso: ~

- **campero_ur10_server**
    - Compilación: si se añaden prefijos a los joints/links en el archivo campero_ur10_server/src/c_ur10_utils.cpp poner la variable USE_PREFIX a 1 y compilar
    - Descripción: Paquete que planifica y ejecuta las trayectorias del robot, hace de intermediario entre las aplicaciones a nivel de usuario y el robot en simulación/real
    - Uso:
    ```bash
        roslaunch campero_ur10_server (draw.launch draw_config:=<file> | teleop.launch -ori:=(0|1))
    ```
    En caso de draw.launch, por defecto draw_config:=../campero_ur10_server/config/ur10_draw.config
    En caso de teleop.launch, por defecto ori:=0, indica que no se añaden restricciones de orientación al robot
    
- **campero_ur10_teleop**
    - Descripción: Paquete para teleoperar el robot mediante teclado, tiene dos modos(cartesiano y articular),envía los conmandos al nodo campero_ur10_server
    - Uso:
    ```bash
        python campero_ur10_teleop.py [-Ms <max_step>] [-s <step>] [-t <time>]
    ```
     por defecto -Ms = 0.1, -s = 0.01 y -t = 1
     
- **description**
    - Descripción: Paquete que contiene archivos urdf/xacro que definen el robot
    - Uso: para cargar una escena en Rviz
    ```bash
        roslaunch campero_ur10_description load_scene.launch [scene_file:=<file>]
    ```
     por defecto scene_file:=campero_ur10_description/config/ws_walls.scene, hay que indicarle la ruta completa al archivo sino falla
     - Uso: lanzar robot ur10 real, requiere lanzar previamente el programa urcap en el ordenador del ur10
    ```bash
        roslaunch campero_ur10_description campero_ur10_bringup.launch [robot_ip:=<ip>] [controller_config_file:=<file_controllers.yaml>] [kinematics_config:=<file_calibration.yaml>] [robot_description_file:=<robot_description.urdf.xacro>]
    ```
     por defecto:
        - robot_ip:=192.168.0.210
        - controller_config_file:=campero_ur10_description/config/ur10_controllers.yaml
        - kinematics_config:=campero_ur10_description/config/ur10_calibration.yaml
        - robot_description_file:=campero_ur10_description)/robots/campero_ur10.urdf.xacro
     
 - **draw_board**
    - Descripción: Paquete que lanza una pizarra para dibujar, la imagen realizada se envía al nodo campero_ur10_server
    - Uso: dos tipos uno dibujando a mano(draw_board_cv) y otro mas preciso(draw_board_turtle)
    ```bash
        python draw_board_cv.py [-s <size>]
        python draw_board_turtle.py [-s <size>]
    ```
     por defecto -s = 512 pixeles, crea una pizarra de SxS pixeles

- **monitoring_tools**
    - Descripción: Paquete con herramientas para monitorear el robot
    - Uso speed_monitor: monitorea la velocidad de los joint
    ```bash
        python speed_monitor.py [-plt]
    ```
     si -plt activado muestra una gráfica con las velocidades de los joints al terminar una trayectoria
    - Uso stop_robot: programa que permite detener la ejecución actual del robot de manera manual o si alguna articulación del robot sobrepasa la velocidad máxima
    ```bash
        python stop_robot.py [-v [<max_speed>]]
    ```
     Una vez lanzado el programa, pulsar Enter para detener al robot y z para salir del programa
     Con el flag -v el programa detiene automaticamente el robot si alcanza una velocidad máxima(max_speed, por defecto 0.5 rad/s), es necesario ejecutar speed_monitor
