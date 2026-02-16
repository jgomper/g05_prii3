from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    target_id_arg = DeclareLaunchArgument(
        'target_id', default_value='22',
        description='Target ArUco ID'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='3',
        description='Robot ArUco ID'
    )

    return LaunchDescription([
        target_id_arg,
        robot_id_arg,



        # 2. Real Navigator (Usa TF para moverse)
        Node(
            package='sprint4_eurobot',
            executable='real_navigator',
            name='real_navigator',
            output='screen',
            parameters=[{
                'target_id': LaunchConfiguration('target_id'),
                'robot_id': LaunchConfiguration('robot_id'),
                'use_sim_time': True
            }]
        )
    ])
