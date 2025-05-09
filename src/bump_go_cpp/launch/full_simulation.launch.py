from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to each package
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    slam_pkg = get_package_share_directory('robotics_slam')
    bumpgo_pkg = get_package_share_directory('bump_go_cpp')

    # SLAM config
    slam_config = os.path.join(slam_pkg, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(slam_pkg, 'rviz', 'async_slam_config.rviz')

    return LaunchDescription([
        # Set TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
            )
        ),

        # Launch SLAM Toolbox with remappings and config
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
            remappings=[
                ('/scan', '/scan')
            ]
        ),

        # Launch RViz preloaded with your config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        # Launch your bump-go FSM node
        Node(
            package='bump_go_cpp',
            executable='bump_go_node',
            name='bump_go',
            output='screen'
        )
    ])

