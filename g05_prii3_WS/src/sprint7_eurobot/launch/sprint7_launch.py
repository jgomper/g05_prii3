import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Argumentos
    kp_arg = DeclareLaunchArgument(
        'kp', default_value='0.005', description='Ganancia proporcional'
    )
    kd_arg = DeclareLaunchArgument(
        'kd', default_value='0.002', description='Ganancia derivativa'
    )
    base_speed_arg = DeclareLaunchArgument(
        'base_speed', default_value='0.2', description='Velocidad base lineal (m/s)'
    )
    threshold_value_arg = DeclareLaunchArgument(
        'threshold_value', default_value='60', description='Valor de umbralización binaria'
    )
    # Tópico de la cámara por defecto
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw', description='Topico imagen raw'
    )

    # Nodo del siguelíneas
    line_follower_node = Node(
        package='sprint7_eurobot',
        executable='line_follower',
        name='line_follower',
        output='screen',
        parameters=[{
            'kp': LaunchConfiguration('kp'),
            'kd': LaunchConfiguration('kd'),
            'base_speed': LaunchConfiguration('base_speed'),
            'threshold_value': LaunchConfiguration('threshold_value'),
            'image_topic': LaunchConfiguration('camera_topic')
        }]
    )

    return LaunchDescription([
        kp_arg,
        kd_arg,
        base_speed_arg,
        threshold_value_arg,
        camera_topic_arg,
        line_follower_node
    ])
