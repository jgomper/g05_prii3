# DEMOSTRACIÓN FINAL EUROBOT - GUÍA DE ARRANQUE

Este documento describe los pasos detallados para ejecutar la demostración completa del robot (Chasis, Brazo y Visión) utilizando un **único archivo launch**, tal como exigen los profesores.

---

## 1. Conexión Inicial con el Robot
En primer lugar, conéctate por SSH a la Jetson del robot habilitando el reenvío de ventanas gráficas (`-X`) para poder ver las ventanas de visión de la cámara:

```bash
ssh -X jetson@192.168.111.125
```

---

## 2. Configuración de Hardware (Permisos USB)
Antes de ejecutar nada de ROS, el sistema operativo necesita permisos para acceder a los controladores motores (brazo) y al Arduino (ventosa). Esto es **crítico** y debe hacerse siempre al encender el robot.

Ejecuta estos comandos en la terminal:

```bash
# 2.1 Configuración de los motores del Brazo
sudo ln -sf /dev/ttyUSB0 /dev/ttyUSB1
sudo chmod 777 /dev/ttyUSB0 /dev/ttyUSB1

# 2.2 Configuración del Arduino de la Ventosa
stty -F /dev/ttyACM0 115200 raw -echo
sudo chmod 777 /dev/ttyACM0
```

---

## 3. Preparación del Entorno (Workspaces)
Como el robot está compuesto por varios paquetes desarrollados en diferentes carpetas, debes "decirle" a ROS 2 dónde encontrar cada uno de ellos. 

En la **misma terminal** donde vas a ejecutar el launch, debes hacer el *source* de los tres entornos:

```bash
source ~/robot_ws/install/setup.bash
source ~/brazo_ws/install/setup.bash
source ~/Proyecto_III/g05_prii3/g05_prii3_WS/install/setup.bash
```
*(Nota: Si usas zsh en lugar de bash, cambia `.bash` por `.zsh`)*

---

## 4. Lanzamiento de la Demostración (One-Click Launch)
Ahora que todo está preparado, puedes lanzar todo el sistema con un único comando. 

Esto arrancará simultáneamente:
- El driver de las ruedas (`diff_car`).
- El driver del brazo (`brazo_pkg`).
- El nodo de Inteligencia Artificial (tu máquina de estados en Python).

Ejecuta:
```bash
# Si el archivo se llama demo_eurobot.launch.py:
ros2 launch /home/rodrigo/Proyecto_III/g05_prii3/g05_prii3_WS/src/Sprint8_eurobot/demo_eurobot.launch.py

# (Si le cambiaste el nombre a sprint8_launch.py, usa ese nombre):
# ros2 launch /home/rodrigo/Proyecto_III/g05_prii3/g05_prii3_WS/src/Sprint8_eurobot/sprint8_launch.py
```

---

## 5. Ejecución de la Misión
Una vez lanzado el comando, verás en la terminal mucha información de inicialización de los nodos.
Tu nodo de la IA se quedará pausado esperando confirmación con el siguiente mensaje:

```text
========================================
  ROBOT EUROBOT - MÁQUINA DE ESTADOS
========================================
Presiona ENTER para iniciar la misión...
```

1. Coloca el robot en la posición de salida de la pista.
2. Comprueba que las ventanas de visión de la cámara (`Vision Original`, etc.) se han abierto en tu pantalla (tardarán unos segundos por la red SSH).
3. **Presiona ENTER** en la terminal. El brazo se moverá a su posición de búsqueda inicial y el robot comenzará a avanzar. ¡Suerte en la demo!
