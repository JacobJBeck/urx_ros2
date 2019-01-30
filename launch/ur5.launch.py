#!/usr/bin/env python3
"""TODO: Docstring"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    """TODO: Docstring"""
    # Find needed folders
    interface_conf = get_package_share_directory('urx_ros2') + '/config/'

    # Create nodes
    interface = Node(package='urx_ros2', node_executable='ur_interface',
                     arguments=['--robot-ip', '192.168.10.247',
                                '--command-topic', '/ur5/joint_command',
                                '--state-topic', '/ur5/joint_states'])
    state_pub = Node(package='robot_state_publisher', node_executable='robot_state_publisher',
                     node_namespace='ur5',
                     arguments=[interface_conf + 'ur5.urdf'])

    return LaunchDescription([interface, state_pub])
