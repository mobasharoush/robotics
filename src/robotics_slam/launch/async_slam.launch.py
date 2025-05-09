from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_config = os.path.join(
        get_package_share_directory('robotics_slam'),
        'config',
        'mapper_params_online_async.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('robotics_slam'),
        'rviz',
        'async_slam_config.rviz'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
            remappings=[('/scan', '/scan')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
