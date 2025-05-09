#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Retrieve package directories
    tb3_pkg       = get_package_share_directory('turtlebot3_gazebo')
    robotics_pkg  = get_package_share_directory('robotics_slam')
    slam_pkg      = get_package_share_directory('slam_toolbox')
    bump_pkg      = get_package_share_directory('bump_go_cpp')

    # Launch Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'empty_world.launch.py')
        )
    )

    groups = []

    # --- Mapper robot group ---
    mapper_ns = 'tb3_map'
    groups.append(
        GroupAction([
            PushRosNamespace(mapper_ns),
            # Spawn mapper robot
            Node(
                package='gazebo_ros', executable='spawn_entity.py',
                arguments=[
                    '-entity', mapper_ns,
                    '-topic', f'/{mapper_ns}/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.01'
                ],
                output='screen'
            ),
            # Robot state publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_pkg, 'launch', 'robot_state_publisher.launch.py')
                ),
                launch_arguments={'namespace': mapper_ns}.items()
            ),
            # Laser filter chain: scan -> scan_filtered
            Node(
                package='laser_filters',
                executable='scan_to_scan_filter_chain',
                name='laser_filter_chain',
                parameters=[
                    os.path.join(robotics_pkg, 'config', 'laser_filter_params.yaml')
                ],
                remappings=[
                    ('scan', f'/{mapper_ns}/scan'),
                    ('scan_filtered', f'/{mapper_ns}/scan_filtered')
                ]
            ),
            # SLAM Toolbox async node on filtered scan
            Node(
                package='slam_toolbox', executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(robotics_pkg, 'config', 'mapper_params.yaml'),
                    {'use_sim_time': True}
                ],
                remappings=[('scan', f'/{mapper_ns}/scan_filtered')]
            ),
        ])
    )

    # --- Obstacle robots group ---
    for i in range(1, 3):
        ns = f'tb3_obs{i}'
        x = 2.0 * ((i-1) % 2)
        y = 2.0 * ((i-1) // 2)
        groups.append(
            GroupAction([
                PushRosNamespace(ns),
                # Spawn obstacle robot
                Node(
                    package='gazebo_ros', executable='spawn_entity.py',
                    arguments=[
                        '-entity', ns,
                        '-topic', f'/{ns}/robot_description',
                        '-x', str(x), '-y', str(y), '-z', '0.01'
                    ],
                    output='screen'
                ),
                # Robot state publisher
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(tb3_pkg, 'launch', 'robot_state_publisher.launch.py')
                    ),
                    launch_arguments={'namespace': ns}.items()
                ),
                # Bump-go node
                Node(
                    package='bump_go_cpp', executable='bump_go_node',
                    name='bump_go',
                    remappings=[
                        ('scan', f'/{ns}/scan'),
                        ('cmd_vel', f'/{ns}/cmd_vel')
                    ]
                ),
            ])
        )

    # Assemble launch description
    ld = LaunchDescription()
    ld.add_action(gazebo)
    for group in groups:
        ld.add_action(group)

    return ld
