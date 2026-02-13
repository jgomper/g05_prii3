import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file para Sprint 5 - Modo Simulación.
    
    Reutiliza la simulación de Gazebo del Sprint 4 pero añade:
    - Simulador de cámara del laboratorio (convierte TFs a Strings)
    - Navegador basado en Strings (en lugar de TFs)
    - Detector de almacenes (Pattern Matching)
    """
    
    # Ruta al paquete del Sprint 4 (para reutilizar Gazebo)
    sprint4_pkg = get_package_share_directory('sprint4_eurobot')
    sprint5_pkg = get_package_share_directory('sprint5_eurobot')
    
    # Argumentos
    target_arg = DeclareLaunchArgument(
        'target',
        default_value='22',
        description='ID del ArUco objetivo para la navegación (20, 21, 22, 23)'
    )
    
    return LaunchDescription([
        target_arg,
        
        # 1. REUTILIZAR LAUNCH DEL SPRINT 4 (Solo Gazebo + Cenital TF)
        # Lanzamos el launch del Sprint 4 pero SIN el navegador
        # (porque ahora usaremos el nuevo navegador basado en Strings)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(sprint4_pkg, 'launch', 'eurobot_rviz_launch.py')
            ])
        ),
        
        # NOTA: Si no tienes eurobot_rviz_launch.py, puedes lanzar directamente
        # todos los componentes del Sprint 4 aquí (Gazebo, cenital_node, TF bridge, RViz)
        # Ver comentario al final de este archivo para código alternativo
        
        # 2. SIMULADOR DE CÁMARA DEL LABORATORIO (NUEVO)
        # Este nodo lee las TFs del Sprint 4 y las convierte a Strings
        Node(
            package='sprint5_eurobot',
            executable='lab_camera_simulator',
            output='screen',
            name='lab_camera_simulator'
        ),
        
        # 3. NAVEGADOR BASADO EN STRINGS (NUEVO)
        Node(
            package='sprint5_eurobot',
            executable='string_navigator',
            output='screen',
            name='string_navigator',
            arguments=[LaunchConfiguration('target')]
        ),
        
        # 4. DETECTOR DE ALMACENES (NUEVO - PBI 5.3)
        Node(
            package='sprint5_eurobot',
            executable='warehouse_detector',
            output='screen',
            name='warehouse_detector',
            parameters=[{
                'min_area': 1000,
                'max_area': 50000,
                'square_tolerance': 0.15
            }]
        ),
    ])

