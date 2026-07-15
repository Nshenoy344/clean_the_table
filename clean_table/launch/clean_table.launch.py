from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clean_table',
            executable='marker_lookup_service',
            name='marker_lookup_service',
            output='screen',
        ),
        Node(
            package='clean_table',
            executable='drive_distance_server',
            name='drive_distance_server',
            output='screen',
        ),
        Node(
            package='clean_table',
            executable='align_to_marker_server',
            name='align_to_marker_server',
            output='screen',
        ),
        Node(
            package='clean_table',
            executable='pick_place_server',
            name='pick_place_server',
            output='screen',
        ),
        Node(
            package='clean_table',
            executable='orchestrator',
            name='orchestrator',
            output='screen',
        ),
    ])
