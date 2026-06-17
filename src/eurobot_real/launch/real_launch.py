import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'eurobot_real'
    pkg_share = get_package_share_directory(package_name)
    
    # Archivo de configuración de RViz (si existe, sino usa default)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'real_view.rviz')

    # Argumento para el ID del objetivo
    target_arg = DeclareLaunchArgument(
        'target',
        default_value='22',
        description='ID del ArUco objetivo'
    )

    return LaunchDescription([
        target_arg,

        # 1. DRIVER DE CÁMARA (USB CAM)
        # Asegúrate de tener instalado: sudo apt install ros-<distro>-usb-cam
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'image_width': 640,
                'image_height': 480,
                'camera_name': 'overhead_camera'
            }],
            output='screen'
        ),

        # 2. PUENTE TF ESTATICO (CRÍTICO)
        # Define la relación entre el marcador que lleva el robot (ID 1) y el centro del robot (base_footprint).
        # Ajusta estos valores según la posición real de la pegatina.
        # Formato: x y z yaw pitch roll frame_id child_frame_id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'aruco_1', 'base_footprint'],
            output='screen'
        ),

        # 3. NODO DE VISIÓN REAL
        Node(
            package='eurobot_real',
            executable='real_vision_node',
            name='real_vision_node',
            output='screen'
        ),

        # 4. NODO DE NAVEGACIÓN REAL
        Node(
            package='eurobot_real',
            executable='real_navigator',
            name='real_navigator',
            output='screen',
            arguments=[LaunchConfiguration('target')]
        ),

        # 5. RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
