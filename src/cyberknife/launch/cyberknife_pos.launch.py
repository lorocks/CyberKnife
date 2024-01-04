#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindExecutable


def generate_launch_description():
    desc = LaunchDescription()

    publish_rate = LaunchConfiguration('publish_rate', default='5.0')

    # Launch Decription to Spawn Robot Model 
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cyberknife'), 'launch',
                         'spawn_cyberknife_pos.launch.py'),
        )
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                          'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'publish_rate': publish_rate,
        }.items(),
    )

    # Spawn ROBOT Set Gazebo (Does not spwan robot only communicates with the Gazebo Client)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'cyberknife', 
                   '-x', '0.0', '-y', '0.0', '-z', '0.00001',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                   '-topic', '/robot_description'
                   ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["broadcaster"],
    )

    joint_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_velocities"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_control_spawnning = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_velocity_controller],
        )
    )

    initial_velocity = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub /joint_velocities/commands std_msgs/msg/Float64MultiArray "{data: [0.0,0.0,0.0,0.0,0.0,0.0],layout: {dim:[], data_offset: 1"}} '
        ]],
        shell = True
    )

# topic pub /joint_velocities/commands std_msgs/msg/Float64MultiArray "{data: [1.0,0.0,0.0,0.0,0.0,0.0],layout: {dim:[], data_offset: 1"}}


    desc.add_action(spawn_robot_world)
    desc.add_action(gazebo)
    desc.add_action(spawn_robot)
    desc.add_action(joint_state_publisher_node)
    # desc.add_action(joint_state_broadcaster)
    # desc.add_action(joint_velocity_controller)
    desc.add_action(delay_controller)
    desc.add_action(delay_control_spawnning)
    # desc.add_action(initial_velocity)
    
    # Launch Description 
    return desc
