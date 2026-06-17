import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Master Launch File para la demostración de Eurobot.
    Arranca los motores del chasis, los motores del brazo y el código principal de visión.
    """
    
    # 1. Lanzar los motores del chasis
    chasis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('diff_car'), 'launch', 'diff_car.launch.py')
        )
    )

    # 2. Lanzar los motores del brazo
    brazo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('brazo_pkg'), 'launch', 'bringup.launch.py')
        )
    )

    # 3. Lanzar el nodo de Python con la Máquina de Estados (IA)
    ruta_script = '/home/rodrigo/Proyecto_III/g05_prii3/g05_prii3_WS/src/Sprint8_eurobot/SPRINT8_FSM_DEFINITIVO.py'
    fsm_node = ExecuteProcess(
        cmd=['python3', ruta_script],
        output='screen'
    )

    return LaunchDescription([
        chasis_launch,
        brazo_launch,
        fsm_node
    ])
