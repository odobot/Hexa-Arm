#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import load_python_launch_file_as_module
from ament_index_python import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Convert xacro to urdf string and set robot_description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('hexaarm_description'),
            'urdf',
            'hexaarm.urdf.xacro'
        ])
    ])
    robot_description_parameters = {'robot_description': robot_description_content}

    # Path to the SRDF file
    robot_description_semantic_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('hexaarm_moveit'),
            'config',
            'hexaarm.srdf'
        ])
    ])

    robot_description_semantic_parameters = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory("hexaarm_moveit_servo"), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    load_yaml = getattr(mod, 'load_yaml')
    servo_yaml = load_yaml('hexaarm_moveit_servo', "config/hexaarm_moveit_servo_config.yaml")
    servo_yaml['move_group_name'] = 'arm'

    servo_params = {"moveit_servo": servo_yaml}

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("hexaarm_moveit_servo"),
        'rviz',
        'rviz_servo.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description_parameters, robot_description_semantic_parameters],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Controller spawner for joint control
    load_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager'
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                'hexaarm_controller',
                '--controller-manager', '/controller_manager'
            ],
        ),
    ]

    # Container for MoveIt Servo and other nodes
    container = ComposableNodeContainer(
        name='hexaarm_servo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                parameters=[robot_description_parameters, robot_description_semantic_parameters],
            ),
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='static_tf2_broadcaster',
                parameters=[{'child_frame_id': 'base_link', 'frame_id': 'world'}],
            ),
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',
                name='servo_server',
                parameters=[servo_params, robot_description_parameters, robot_description_semantic_parameters],
            ),
        ],
        output='screen',
    )

    # return [container] #+ load_controllers
    return [rviz_node, container] #+ load_controllers

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
