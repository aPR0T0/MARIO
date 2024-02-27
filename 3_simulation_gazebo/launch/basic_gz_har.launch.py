#!/usr/bin/python3

# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mario_bot = get_package_share_directory('simulation_gazebo')
    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "simulation_gazebo"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
    )    
    # model launch
    mario = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mario_bot, 'launch', 'mario2.launch.py'),
        )
    )   
    return LaunchDescription([  
        mario,
        gazebo
    ])