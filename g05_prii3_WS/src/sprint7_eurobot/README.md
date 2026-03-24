# Sprint 7: Siguelíneas (Line Follower)

Paquete que implementa un nodo autónomo de ROS2 para realizar el seguimiento de una línea oscura sobre un fondo claro, usando la cámara a bordo del JetBot y Visión Artificial (OpenCV). Cumple con los requisitos del PBI 7.1 y PBI 7.2.

##Rviz robot

ros2 run rviz2 rviz2


## Instrucciones y Lanzamiento Unificado (PBI 7.2)

### 0. Levantar Robot
Abre un terminal, conéctate al robot y arranca sus drivers base:
```bash
ssh -X jetbot@192.168.111.83

export ROS_DOMAIN_ID=55
cd ~/jetbot_ws
source install/setup.bash
ros2 launch jetbot_pro_ros2 jetbot.py
```

### 1. Despliegue en el Robot
Desde tu PC principal y situado en el workspace, copia este paquete al robot enviándolo por red local (SSH/SCP).
```bash
scp -r ~/Proyecto_III/g05_prii3/g05_prii3_WS/src/sprint7_eurobot jetbot@192.168.111.83:~/jetbot_ws/src/
```

### 2. Compilación
Abre un terminal SSH en el JetBot y compila el paquete `sprint7_eurobot`.
```bash
ssh -X jetbot@192.168.111.83
cd ~/jetbot_ws
colcon build --packages-select sprint7_eurobot --symlink-install
source install/setup.bash
```

### 3. Ejecución
Coloca el JetBot sobre la pista o el circuito al inicio de la línea oscura y lanza la misión:

```bash
# Lanzar el algoritmo de siguelineas (se asume que la cámara y base ya publican, si no, añádelos a tu cadena de start)
export ROS_DOMAIN_ID=55
cd ~/jetbot_ws
source install/setup.bash
ros2 launch sprint7_eurobot sprint7_launch.py
```


### SOLUCION ERROR CAMARA
sudo systemctl restart nvargus-daemon

unset DISPLAY
