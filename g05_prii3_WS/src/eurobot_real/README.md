# Comandos de Ejecución - Eurobot Real

## 1. Robot (Jetbot)
**Objetivo:** Encender motores y escuchar `/cmd_vel`.
*Ejecutar en el Jetbot (vía SSH o terminal directa):*

```bash
# Lanza el driver base de tu robot
# (Asegúrate de que este nodo se suscriba a /cmd_vel)
ros2 launch jetbot_ros jetbot_bringup.launch.py
```
*(Nota: Ajusta el comando anterior según el paquete específico instalado en tu Jetbot)*

---

## 2. PC Central (Laptop)
**Objetivo:** Procesar visión y enviar comandos.
*Ejecutar en el PC, desde la raíz del workspace:*

### A. Compilar (Preparación)
```bash
cd ~/Proyecto_III/g05_prii3/g05_prii3_WS
colcon build --packages-select eurobot_real --symlink-install
source install/setup.bash
```

### B. Lanzar Misión
Este comando inicia: Cámara USB + Nodo Visión + Navegador + RViz.

```bash
# Ir al ArUco 22
ros2 launch eurobot_real real_launch.py target:=22
```

### C. Opciones
Cambiar el objetivo:
```bash
ros2 launch eurobot_real real_launch.py target:=20
ros2 launch eurobot_real real_launch.py target:=23
```
