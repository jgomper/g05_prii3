from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns', default_value='/',
        description='Namespace ROS2 del robot (ej: robot6)'
    )

    avance_vel_arg = DeclareLaunchArgument(
        'avance_vel', default_value='0.15',
        description='Velocidad lineal por lado [m/s]'
    )

    avance_duracion_arg = DeclareLaunchArgument(
        'avance_duracion', default_value='4.0',
        description='Duración del avance por lado [s]'
    )

    giro_vel_arg = DeclareLaunchArgument(
        'giro_vel', default_value='0.50',
        description='Velocidad angular para girar 90° [rad/s]'
    )

    giro_duracion_arg = DeclareLaunchArgument(
        'giro_duracion', default_value='3.14',
        description='Duración del giro de 90° [s]  (pi/2 / giro_vel)'
    )

    return LaunchDescription([
        robot_ns_arg,
        avance_vel_arg,
        avance_duracion_arg,
        giro_vel_arg,
        giro_duracion_arg,

        Node(
            package='sprint6_eurobot',
            executable='fsm_navigator',
            name='fsm_navigator',
            namespace=LaunchConfiguration('robot_ns'),
            output='screen',
            parameters=[{
                'avance_vel':      LaunchConfiguration('avance_vel'),
                'avance_duracion': LaunchConfiguration('avance_duracion'),
                'giro_vel':        LaunchConfiguration('giro_vel'),
                'giro_duracion':   LaunchConfiguration('giro_duracion'),
            }]
        )
    ])
