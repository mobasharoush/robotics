from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bump_go_cpp',
            executable='bump_go_node',
            name='bump_go',
            output='screen'
        )
    ])
