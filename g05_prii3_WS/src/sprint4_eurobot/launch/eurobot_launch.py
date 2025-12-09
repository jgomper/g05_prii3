import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'sprint4_eurobot'
    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'world', 'eurobot.world')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'eurobot_config.rviz') # Configuración personalizada

    # --- CORRECCIÓN IMPORTANTE ---
    # Si la variable ya existe, añadimos nuestra ruta al final.
    if 'GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['GAZEBO_RESOURCE_PATH'] = os.environ['GAZEBO_RESOURCE_PATH'] + ':' + pkg_share
    else:
        # SI NO EXISTE, debemos incluir la ruta por defecto de Gazebo (/usr/share/gazebo-11)
        # Si no hacemos esto, Gazebo no encuentra sus shaders y se cierra.
        os.environ['GAZEBO_RESOURCE_PATH'] = '/usr/share/gazebo-11:' + pkg_share
    # -----------------------------

    # Set TURTLEBOT3_MODEL if not already set
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Locate turtlebot3_gazebo
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    
    # Add turtlebot3 models to GAZEBO_MODEL_PATH
    turtlebot3_models_path = os.path.join(turtlebot3_gazebo_pkg, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + turtlebot3_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = turtlebot3_models_path

    # --- ARGUMENTOS DE LANZAMIENTO ---
    target_arg = DeclareLaunchArgument(
        'target',
        default_value='22',
        description='ID del ArUco objetivo para la navegación (20, 21, 22, 23)'
    )

    # --- RANDOM SPAWN LOGIC ---
    # Límites del tablero (aprox 3x2m) con margen de seguridad
    # X: [-1.35, 1.35]
    # Y: [-0.85, 0.85]
    rand_x = random.uniform(-1.35, 1.35)
    rand_y = random.uniform(-0.85, 0.85)
    rand_yaw = random.uniform(-3.14, 3.14)
    print(f"--- SPAWNING ROBOT AT: X={rand_x:.2f}, Y={rand_y:.2f}, YAW={rand_yaw:.2f} ---")
    # --------------------------

    return LaunchDescription([
        target_arg,

        # 1. GAZEBO
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. SPAWN ROBOT (RANDOM POSE)

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle',
                '-file', os.path.join(pkg_share, 'models', 'waffle_aruco', 'model.sdf'),
                '-x', str(rand_x),
                '-y', str(rand_y),
                '-z', '0.01',
                '-Y', str(rand_yaw)
            ],
            output='screen'
        ),

        # 3. PUENTE TF VISUAL
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.5708', '0', '0', 'aruco_1', 'robot_visual_link'],
            output='screen'
        ),

        # 4. NODO DE VISIÓN (CENITAL)
        Node(
            package='sprint4_eurobot',
            executable='cenital_node',
            output='screen'
        ),

        # 5. RVIZ2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # 6. NODO DE NAVEGACIÓN (VISUAL NAVIGATOR)
        Node(
            package='sprint4_eurobot',
            executable='visual_navigator',
            output='screen',
            arguments=[LaunchConfiguration('target')]
        ),
    ])