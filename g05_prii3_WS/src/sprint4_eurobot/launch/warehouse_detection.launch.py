import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'sprint4_eurobot'
    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'world', 'eurobot.world')

    # --- CONFIGURACIÓN DE GAZEBO ---
    # Es crucial añadir la ruta de nuestros modelos y materiales al GAZEBO_RESOURCE_PATH
    if 'GAZEBO_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GAZEBO_RESOURCE_PATH'] + ':' + pkg_share
    else:
        gz_resource_path = '/usr/share/gazebo-11:' + pkg_share

    return LaunchDescription([
        # 1. Configurar Entorno
        SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=gz_resource_path),

        # 2. Lanzar Gazebo (Solo Mundo, SIN Cliente GUI ahorra recursos si solo quieres ver la ventana de OpenCV)
        # Pero el usuario pidió "se abra el mundo de gazebo", así que lanzamos 'gazebo' completo.
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 3. Lanzar Nodo de Detección (Pattern Matching)
        Node(
            package='sprint4_eurobot',
            executable='pattern_matching',
            name='pattern_matching_node',
            output='screen',
            parameters=[{
                'image_topic': '/overhead_camera/image_raw',
                'show_gui': True,  # Mostrar ventana OpenCV
                'match_threshold': 0.45
            }]
        ),
    ])
