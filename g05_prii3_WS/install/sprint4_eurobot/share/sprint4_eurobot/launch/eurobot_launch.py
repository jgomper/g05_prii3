import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

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

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])