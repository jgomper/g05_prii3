# Sprint 5 - Prueba Laboratorio I3L7

## PBI 5.1: Procesamiento ArUco (0.10 pts)
**Demostrar suscripción y extracción de datos de topics**

### En tu PC:
```bash
# Configurar red
export ROS_DOMAIN_ID=1
ros2 daemon stop && ros2 daemon start

# Ver topics disponibles
ros2 topic list | grep overhead_camera

# Ver datos ArUco robot (debe mostrar JSON con px, py, orientation)
ros2 topic echo /overhead_camera/aruco_3

# Ver datos ArUco objetivo
ros2 topic echo /overhead_camera/aruco_20
```

---

## PBI 5.2: Navegación Autónoma (0.50 pts)
**Demostrar movimiento del robot a marcador objetivo**

### En el JetBot (vía SSH):
```bash
# Lanzar driver del robot con namespace
# IMPORTANTE: El driver debe escuchar en /robot5/cmd_vel
cd ~/jetbot_ws
source install/setup.bash
export ROS_DOMAIN_ID=1

# Si el driver soporta remapping:
ros2 run jetbot_pro_ros2 jetbot --ros-args -r cmd_vel:=robot5/cmd_vel

# O simplemente (depende del setup del robot):
ros2 run jetbot_pro_ros2 jetbot
```

### En tu PC:
```bash
# Compilar
cd ~/Proyecto_III/g05_prii3/g05_prii3_WS
source install/setup.bash

# Ejecutar navegador ROBOT 5 hacia ArUco 22 (cambia 22 por 20, 21, 22, o 23)
ros2 run sprint5_eurobot string_navigator 22 --ros-args -p robot_namespace:=robot5

# En OTRA TERMINAL: Ver comandos enviados al robot
ros2 topic echo /robot5/cmd_vel
```

---

## Debug
```bash
# Ver nodos activos
ros2 node list

# Ver info de topics
ros2 topic info /overhead_camera/aruco_3
```
