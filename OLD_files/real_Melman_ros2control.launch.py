# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    package_name = os.path.join(
        get_package_share_directory('ros2_humanoid_virtual_twin'))

    xacro_file = os.path.join(package_name,
                              'description',
                              'robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_gazebo_link_physics_coefficients': 'false', 'use_gazebo_joint_physics_coefficients': 'false', 'use_URDF_joint_dynamics_coefficients': 'false', 'use_gazebo': 'false'})
    params = {'robot_description': doc.toxml()}

    rviz_config = os.path.join(get_package_share_directory(
        'ros2_humanoid_virtual_twin'), "launch", "melman.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    controller_config = os.path.join(
        get_package_share_directory(
            'ros2_humanoid_virtual_twin'), "controllers", "controllers.yaml"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": doc.toxml()}, controller_config],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )    

    return LaunchDescription([
        controller_manager_node,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller_manager_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_velocity_controller],
            )
        ),

        node_robot_state_publisher,
        rviz_node,
    ])
