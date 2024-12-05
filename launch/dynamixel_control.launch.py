#!/usr/bin/env python3

# Copyright (c) 2024, SENAI CIMATEC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def controllers_node_launch(context, *args, **kwargs):
    controllers_group = LaunchConfiguration(
        'controllers_group').perform(context)
    nodes = []
    if controllers_group == 'trajectory':
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "-c",
                "/controller_manager"
                ],
            output="screen",
        ))
    elif controllers_group == 'position':
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller", "-c", "/controller_manager"],
            output="screen",
        ))
    return nodes


def generate_launch_description():
    use_sim_time = LaunchConfiguration(
                                'use_sim_time',
                                default=True
                                )
    bool_is_real_launch = True
    is_real_launch = LaunchConfiguration(
                                    'is_real_launch',
                                    default=bool_is_real_launch
                                    )

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock'
    )

    controllers_group_arg = DeclareLaunchArgument(
        'controllers_group',
        default_value='trajectory',
        description='Determine the set of controllers to be launched',
        choices=['trajectory', 'position']
    )

    robot_name = "open_manipulator_pro"
    package_name = robot_name + "_description"

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'omp_ros2.rviz'
    )

    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'open_manipulator_pro.urdf.xacro'
    )

    robot_description = {'robot_description': Command(
        ['xacro ', xacro_file, ' is_real_launch:=', is_real_launch])}

    controller_config = os.path.join(
        get_package_share_directory(
            "open_manipulator_pro_control"), "config", "controllers.yaml"
    )

    return LaunchDescription([
        use_sim_time_arg,
        controllers_group_arg,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description, controller_config],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
                ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        OpaqueFunction(function=controllers_node_launch),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                robot_description],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    ])
