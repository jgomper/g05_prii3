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

    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns', default_value='robot6',
        description='Robot Namespace (e.g. robot6)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation time (True for Gazebo, False for Real World)'
    )

    return LaunchDescription([
        target_id_arg,
        robot_id_arg,
        robot_ns_arg,
        use_sim_time_arg,

        Node(
            package='sprint5_eurobot',
            executable='real_navigator',
            name='real_navigator',
            namespace=LaunchConfiguration('robot_ns'),
            output='screen',
            parameters=[{
                'target_id': LaunchConfiguration('target_id'),
                'robot_id': LaunchConfiguration('robot_id'),
                'orientation_offset': -1.57, # CORRECTED OFFSET
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
