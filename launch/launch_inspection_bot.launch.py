# Copyright 2023 Tarun Trilokesh

"""
@file launch_inspection_bot.py
@brief Launch file for the InspectionBot project in ROS 2.

This launch file initializes the Gazebo simulation environment with a custom
world, spawns the TurtleBot3 robot, and starts the SLAM Toolbox for autonomous
navigation and mapping.

@author Tarun Trilokesh
@date 12/12/2023
@version 1.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 

def generate_launch_description():
    """
    @brief Generate the launch description for the InspectionBot project.

    This function sets up the Gazebo simulation with a custom world file,
    spawns the TurtleBot3 robot model, and initializes the SLAM Toolbox
    with a custom configuration for autonomous navigation and mapping.

    @return LaunchDescription object containing the launch setup.
    """
    # Get the directory of your package
    pkg_inspection_bot = get_package_share_directory('inspection_bot')

    # Path to the custom world file
    world_file_path = os.path.join(pkg_inspection_bot, 'worlds', 'industry.world')

    # Path to the custom SLAM Toolbox configuration file
    slam_config_path = os.path.join(pkg_inspection_bot, 'config', 'slam_config.yaml')

    # Launch Gazebo with the custom world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Set the TurtleBot3 model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')

    # TurtleBot3 spawn launch file
    tb3_spawn_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'spawn_turtlebot3.launch.py'
    )

    # Include the TurtleBot3 spawn launch file
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_spawn_launch_file),
        launch_arguments={'model': turtlebot3_model}.items()
    )

    # SLAM Toolbox Node with custom configuration
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path, {'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        spawn_tb3,
        slam_toolbox
    ])
