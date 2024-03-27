#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
import launch_ros.actions
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch_ros.descriptions import ParameterValue
import random

# this is the function launch  system will look for
def generate_launch_description():
    package_description = "simulation_dh"
    rviz_launch = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time' : True}],
            arguments=['-d' + os.path.join(get_package_share_directory(package_description), 'rviz', 'rviz_config.rviz')]
        )
    python_node = Node(
        package=package_description,
        executable='broadcaster.py'
    )
    return LaunchDescription([  
        # load_joint_position_controller,
        python_node,
        rviz_launch,
        # joint_state_publisher,
    ])