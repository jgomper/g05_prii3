import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'sprint4_eurobot'
    pkg_share = get_package_share_directory(package_name)
    
    # Path to the existing launch file
    eurobot_launch_path = os.path.join(pkg_share, 'launch', 'eurobot_launch.py')
    
    # Path to the RViz config file
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'eurobot.rviz')

    return LaunchDescription([
        # Include the main simulation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(eurobot_launch_path)
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
