# Guía de Implementación - Sprint 4 Eurobot - 4/12/2025

Este documento resume los pasos técnicos realizados para responder a las preguntas del profesor.

## 1. El Tablero y el Marcador de Origen (ArUco Suelo)

**¿Cómo has configurado el tablero?**
*   **Material**: Creé un script de material (`eurobot.material`) con la textura del mapa. Tuve que rotarla 90º y ajustar la escala a `1.0 1.0` para que encajara perfecta en el rectángulo de 3x2 metros sin repetirse.
*   **Marcador de Origen**: Añadí un modelo estático (`origin_marker`) en el archivo `.world`. Es una caja plana (20x20cm) con la textura del ArUco ID 1, colocada exactamente en las coordenadas del Nido (`X: -1.175, Y: 0.7`).

## 2. Añadir el Robot (TurtleBot3)

**¿Cómo has metido el robot en la simulación?**
*   Usé el nodo `spawn_entity.py` del paquete `gazebo_ros` dentro del archivo de lanzamiento (`eurobot_launch.py`).
*   **Problema del robot invisible**: Gazebo no encontraba los modelos por defecto. Lo solucioné añadiendo la ruta `turtlebot3_gazebo/models` a la variable de entorno `GAZEBO_MODEL_PATH` dentro del script de lanzamiento.

## 3. Customización del Robot (Marcador ArUco ID 1)

**¿Cómo le has puesto el marcador encima al robot?**
*   **Estrategia**: En lugar de modificar los archivos originales del sistema (que es mala práctica), creé un **modelo personalizado** local llamado `waffle_aruco`.
*   **Técnica del `<include>`**: Mi archivo SDF (`model.sdf`) importa el robot original usando la etiqueta `<include><uri>model://turtlebot3_waffle</uri></include>`.
*   **Unión Fija**: Añadí un `joint` de tipo `fixed` que une la base del robot (`base_footprint`) con una nueva pieza (`aruco_box`).
*   **Resultado**: Al ser una unión fija, el marcador es parte física del robot y se mueve solidariamente con él.

**¿Cómo haces que salga ese robot y no el normal?**
*   Modifiqué el `eurobot_launch.py` para que el nodo `spawn_entity` cargue mi archivo `waffle_aruco/model.sdf` en lugar del modelo por defecto.
