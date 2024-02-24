import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ros2_humanoid_virtual_twin'
    file_subpath = 'description/robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': False}] # add other parameters here if required
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'launch/melman.rviz')],
    )
    #Configuration of node responsible for moving the leg
    node_move_leg = Node(
        package='ros2_humanoid_virtual_twin',
        executable='move_leg',
        name='move_leg',
        output='screen',
        parameters=[{"frequency": 2.0, "amplitude": 0.5}]  # Set the frequency parameter here
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz,
        node_move_leg,
    ])
