#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch_ros.descriptions import ParameterValue

import xacro

# this is the function launch  system will look for
def generate_launch_description():
    ####### DATA INPUT ##########
    urdf_file = 'manipulator.sdf'
    #xacro_file = "box_bot.xacro"robot_description
    package_description = "simulation_gazebo"
    ####### DATA INPUT END ##########
    config = os.path.join( get_package_share_directory('simulation_gazebo'),
    'config',
    'manipulator.yaml'
    )
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    doc = xacro.parse(open(robot_desc_path))
    params = {'robot_description': doc.toxml()}
    print("Fetching URDF ==>")
    robot_description_content = Command(['xacro ',robot_desc_path])
    robot_description = {"robot_description": robot_description_content}
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )


    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params, config]
    )

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "mario"
    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                    '-topic', '/robot_description'
                    ],
        parameters=[{'string': doc.toxml(),
                     'name': robot_base_name,
                     'allow_renaming': True}],
    )
    # load_joint_position_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_controllers'],
    # output='screen'
    # )
    # create and return launch description object
    delay_joint_state_broadcaster_spawner_after_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    return LaunchDescription([  
        # load_joint_position_controller,
        control_node,
        robot_state_publisher_node,
        spawn_robot,
        delay_joint_state_broadcaster_spawner_after_spawn_robot,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ])