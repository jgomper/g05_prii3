import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file para Sprint 5 - Modo Robot Real (Laboratorio I3L7).
    
    Este launch se ejecuta en el PC Central (laptop) cuando estás en el laboratorio.
    
    Asume que:
    - La cámara cenital del laboratorio ya está publicando topics de ArUcos
    - El Jetbot está corriendo su driver base (suscrito a /cmd_vel)
    
    Solo lanza:
    - Navegador basado en Strings (lee datos de la cámara real)
    - Detector de almacenes (opcional)
    - RViz para visualización
    """
    
    sprint5_pkg = get_package_share_directory('sprint5_eurobot')
    
    # Argumentos
    target_arg = DeclareLaunchArgument(
        'target',
        default_value='22',
        description='ID del ArUco objetivo para la navegación (20, 21, 22, 23)'
    )
    
    use_warehouse_detector_arg = DeclareLaunchArgument(
        'use_warehouse_detector',
        default_value='false',
        description='Activar detector de almacenes (true/false)'
    )
    
    return LaunchDescription([
        target_arg,
        use_warehouse_detector_arg,
        
        # 1. NAVEGADOR BASADO EN STRINGS
        # Lee del topic real de la cámara del laboratorio
        Node(
            package='sprint5_eurobot',
            executable='string_navigator',
            output='screen',
            name='string_navigator',
            arguments=[LaunchConfiguration('target')],
            parameters=[{
                # Aquí puedes sobrescribir el nombre del topic si es diferente
                # 'aruco_topic': '/cenital_camera/arucos'  # Ejemplo
            }]
        ),
        
        # 2. DETECTOR DE ALMACENES (Opcional - PBI 5.3)
        # NOTA: Comentado por defecto. Descomentar si necesitas usarlo.
        # Node(
        #     package='sprint5_eurobot',
        #     executable='warehouse_detector',
        #     output='screen',
        #     name='warehouse_detector',
        #     condition=IfCondition(LaunchConfiguration('use_warehouse_detector')),
        #     parameters=[{
        #         'min_area': 1000,
        #         'max_area': 50000,
        #         'square_tolerance': 0.15
        #     }]
        # ),
        
        # 3. RViz para visualización (Opcional)
        # Puedes usar la configuración del Sprint 4 o crear una nueva
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(sprint5_pkg, 'rviz', 'sprint5_config.rviz')],
        #     output='screen'
        # ),
    ])

# ============================================================================
# INSTRUCCIONES DE USO EN LABORATORIO
# ============================================================================
#
# PASO 1: En el Jetbot (vía SSH)
# -------------------------------
# ros2 launch jetbot_ros jetbot_bringup.launch.py
#
# PASO 2: En el PC Central (laptop)
# ----------------------------------
# cd ~/Proyecto_III/g05_prii3/g05_prii3_WS
# colcon build --packages-select sprint5_eurobot --symlink-install
# source install/setup.bash
#
# # Navegar hacia ArUco 22
# ros2 launch sprint5_eurobot sprint5_real_launch.py target:=22
#
# PASO 3: Verificar conexión de red
# ----------------------------------
# Asegúrate de que tu PC puede ver los topics publicados por la cámara:
# ros2 topic list | grep aruco
#
# Si no ves los topics, verifica:
# - Configuración ROS_DOMAIN_ID (debe ser la misma en todos los dispositivos)
# - Red WiFi (todos deben estar en la misma red)
# - Firewall del PC (puede bloquear tráfico multicast de ROS2)
#
# PASO 4: Verificar datos de la cámara
# -------------------------------------
# ros2 topic echo /aruco_data_string
#
# Deberías ver mensajes como: "id:20;x:1.54;y:0.43"
#
