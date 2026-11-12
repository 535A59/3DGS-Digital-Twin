from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_to_octomap',
            executable='cloud_to_octomap_node',
            name='cloud_to_octomap',
            parameters=[
                {'frame_id': 'world'},
                {'resolution': 0.05}
            ],
            output='screen'
        )
    ])