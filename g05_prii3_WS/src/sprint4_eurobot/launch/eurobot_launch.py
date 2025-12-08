import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'sprint4_eurobot'
    pkg_share = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share, 'world', 'eurobot.world')

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

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle',
                '-file', os.path.join(pkg_share, 'models', 'waffle_aruco', 'model.sdf'),
                '-x', '0.9',
                '-y', '0.8',
                '-z', '0.01',
                '-Y', '-1.57'
            ],
            output='screen'
        ),

        # --- PUENTE TF VISUAL ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.5708', '0', '0', 'aruco_1', 'robot_visual_link'],
            output='screen'
        ),

        # --- NODO DE VISIÓN (CENITAL) ---
        Node(
            package='sprint4_eurobot',
            executable='cenital_node',
            output='screen'
        ),

    ])