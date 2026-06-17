# Proyecto III - Robótica Móvil (Eurobot)

Bienvenido al repositorio oficial del Grupo 05 correspondiente a la asignatura anual de Proyecto III.

Este repositorio documenta el trabajo íntegro desarrollado a lo largo del curso académico para la concepción, diseño e implementación de un sistema robótico autónomo fundamentado en la normativa de la competición de robótica Eurobot. Todo el proyecto ha sido orquestado sobre el middleware ROS 2 (Robot Operating System), garantizando una arquitectura distribuida, modular y escalable.

El desarrollo abarca el ciclo de vida completo de un proyecto de ingeniería robótica: desde las fases iniciales de diseño algorítmico y simulación en entornos virtuales (Gazebo), hasta la integración final de los diferentes subsistemas de hardware y software en un entorno físico. El robot real consta de una plataforma móvil diferencial (Jetbot), un brazo robótico manipulador para la interacción con los elementos de la pista y un sistema de visión artificial basado en cámara.

---

## Fases del Desarrollo (Sprints)

La asignatura se ha estructurado siguiendo una metodología de trabajo incremental, dividiendo los objetivos en Sprints sucesivos. Esta aproximación ha permitido validar progresivamente las funcionalidades, incrementando la complejidad del sistema de forma iterativa y segura.

### Sprint 4: Recreación de Entornos y Algoritmia de Visión (Simulación)
Esta primera etapa supuso la toma de contacto con el entorno de simulación Gazebo y el diseño de la lógica fundamental de navegación y percepción. Se recreó el tablero de la competición y se implementaron algoritmos de visión por computador para la detección de elementos de interés (almacenes y marcadores ArUco). Durante este sprint, se establecieron las bases para el seguimiento de balizas visuales, asegurando que los modelos matemáticos y el procesado de imágenes fuesen lo suficientemente robustos antes de implementarse en el robot físico.

### Sprint 5: Navegación de la Plataforma Real (Primer Despliegue)
El objetivo central de esta fase fue la transferencia del entorno simulado al entorno físico. Se configuró el middleware ROS 2 directamente sobre la placa de procesamiento del Jetbot y se comprobó la comunicación a nivel de hardware. Se diseñaron nodos capaces de publicar y suscribirse a tópicos esenciales como las velocidades de los motores (`/cmd_vel`). Este hito garantizó que el sistema de tracción del robot respondiese de forma fiable a los comandos de velocidad, sentando los cimientos de la navegación física autónoma.

### Sprint 6: Autonomía Estructurada mediante Máquinas de Estados (FSM)
Con la movilidad de la plataforma asegurada, el siguiente paso fue dotar al sistema de capacidad de decisión autónoma y gestión de objetivos a largo plazo. Para ello, se integró una Máquina de Estados Finitos (FSM) apoyada en la librería de Python `transitions`. Esta arquitectura permitió modelar el comportamiento del robot en diferentes etapas lógicas discretas: estados de patrullaje, búsqueda visual de objetivos, aproximación de precisión y rutinas de control del entorno. El sistema dejó de ser un simple ejecutor de comandos reactivos para convertirse en un agente capaz de gestionar secuencias de misiones estructuradas.

### Sprint 7: Control Dinámico y Seguimiento de Trayectorias (Siguelíneas)
Esta fase se dedicó a perfeccionar el sistema de navegación local del robot mediante visión computacional. Se implementó un algoritmo de procesamiento de imagen que, combinando espacios de color (HSV/Chroma) y técnicas de filtrado dinámico, lograba aislar las líneas de seguimiento trazadas en el circuito. A partir de la información visual procesada, se implementó un controlador de la cinemática del robot para corregir de forma continua su trayectoria, asegurando movimientos fluidos en curvas e inmunidad frente al ruido visual o las interferencias cromáticas (decoraciones) del entorno.

### Sprint 8: Integración Completa de Subsistemas y Demostración Final
Como clímax de la asignatura, este sprint abordó el reto de la integración de sistemas de naturaleza heterogénea. Se unificaron bajo la misma lógica de estados la navegación autónoma del chasis, la actuación mecánica (controladores seriales del brazo robótico) y el accionamiento neumático (control de relés mediante Arduino para la ventosa de agarre). Además, se desarrolló una arquitectura de inicialización centralizada mediante un único archivo de ejecución (`demo_eurobot.launch.py`). Esta solución de *One-Click Launch* arranca automáticamente todos los nodos del sistema, sincroniza los drivers de hardware en segundo plano y deja la FSM a la espera de una única orden externa por terminal, permitiendo una demostración final fluida.

### Paquetes Adicionales y Herramientas
* **Paquete `eurobot_real`**: Módulo enfocado en el despliegue del sistema utilizando una cámara cenital independiente. Proporciona posicionamiento global y seguimiento de marcadores ArUco desde una perspectiva superior, sirviendo como arquitectura alternativa para navegación en coordenadas globales.
* **Directorio `Robotica_movil`**: Contiene implementaciones tempranas, pruebas de concepto y código base desarrollado en las primeras sesiones prácticas de la asignatura. Sirve como registro de la evolución técnica y asimilación de conceptos del equipo.

---

## Stack Tecnológico y Hardware

La consecución de los objetivos planteados requirió el dominio del siguiente conjunto de tecnologías:

* **Arquitectura de Software y Middleware:** ROS 2 (Humble / Foxy), garantizando comunicaciones modulares basadas en el estándar DDS.
* **Lenguajes de Programación:** Desarrollo de la lógica de alto nivel y algoritmos en Python, así como scripts de configuración o bajo nivel en C/C++.
* **Plataformas de Hardware:** Sistema integrado compuesto por una NVIDIA Jetbot (procesamiento principal y hardware de tracción), microcontroladores Arduino (gestión electrónica periférica) y un brazo manipulador robótico de múltiples grados de libertad.
* **Procesamiento de Visión:** Empleo intensivo de la biblioteca OpenCV para operaciones matriciales, conversión de espacios de color, segmentación, detección de contornos (Pattern Matching) y extracción de descriptores ArUco.
* **Entornos de Simulación:** Framework Gazebo, empleado como banco de pruebas para el prototipado rápido de algoritmos sin riesgo para la integridad física de los componentes.

---

## Estructura del Workspace

El conjunto de paquetes de ROS 2 que componen los distintos Sprints se encuentra organizado dentro del siguiente directorio de trabajo (Workspace):
`/g05_prii3_WS/src/`

Para replicar el entorno de desarrollo y ejecución, es necesario compilar el proyecto utilizando la herramienta estándar `colcon`. Cada uno de los módulos de software incluye sus respectivos manuales detallados (archivos `README.md` o `COMANDOS.md`), los cuales indican los requisitos de construcción específicos (`colcon build`) y los comandos de inicialización (`ros2 launch` y `ros2 run`) requeridos para operar cada segmento del proyecto.
