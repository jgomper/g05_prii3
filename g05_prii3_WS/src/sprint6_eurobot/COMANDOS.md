--------------------Terminal para conectarse al robot

ping 192.168.111.83

ssh -X jetbot@192.168.111.83

export ROS_DOMAIN_ID=1
cd ~/jetbot_ws
source install/setup.bash
ros2 run jetbot_pro_ros2 jetbot

---------Terminal 2 Robot (EJECUTAR NAVEGACION FSM Sprint 6)
cd ~/jetbot_ws
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch sprint6_eurobot real_robot.launch.py robot_ns:=/

---------Ajustar parámetros del cuadrado (opcional)
ros2 launch sprint6_eurobot real_robot.launch.py robot_ns:=/ avance_duracion:=3.0
ros2 launch sprint6_eurobot real_robot.launch.py robot_ns:=/ giro_duracion:=3.5

---------EJECUTAR FSM SEGUIDOR ARUCO (PBI 6.1)
cd ~/jetbot_ws
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 run sprint6_eurobot aruco_follower

---------Ajustar parámetros del seguidor (opcional)
ros2 run sprint6_eurobot aruco_follower --ros-args -p target_id:=17 -p target_size:=150

-----Meter codigo en el robot 

cd ~/Proyecto_III/g05_prii3/g05_prii3_WS/src
scp -r sprint6_eurobot jetbot@192.168.111.83:~/jetbot_ws/src/

-----Instalar transitions en el robot (solo primera vez)
pip install transitions

-----compile (EN EL ROBOT)
cd ~/jetbot_ws
colcon build --packages-select sprint6_eurobot --symlink-install
source install/setup.bash

-----------------------Terminal portatil (DEBUG / MONITORIZACION)

export ROS_DOMAIN_ID=1
ros2 topic list
ros2 topic echo /cmd_vel

-----------------------ESTADOS FSM (Cuadrado)

INIT    - Estado inicial. Espera 1s y arranca la secuencia
AVANZAR - El robot avanza recto (avance_duracion segundos)
GIRAR   - El robot gira ~90 grados (giro_duracion segundos)
DONE    - Los 4 lados completados. Robot parado (estado final)

-----------------------ESTADOS FSM SEGUIDOR ARUCO + CUADRADO (PBI 6.1)

INIT             - Estado inicial. Espera 2s y arranca la secuencia
BUSCAR           - Gira buscando el ArUco 17
ACERCAR          - Avanza hacia el ArUco, centrandose en el
GIRAR_180        - Cuando pierde el ArUco (muy cerca), gira 180 grados (pi rad)
CUADRADO_AVANZAR - Avanza un lado del cuadrado (primer lado tras giro 180)
CUADRADO_GIRAR   - Gira 90 grados (pi/2 rad) para el siguiente lado
DONE             - Secuencia completada (4 lados del cuadrado)

Calculos de tiempo:
  - Giro 180 = pi/0.8 = 3.93 segundos
  - Giro 90 = (pi/2)/0.8 = 1.96 segundos

