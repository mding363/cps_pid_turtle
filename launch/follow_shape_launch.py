#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to start the complete turtle control system.

    This will start:
    1. Turtlesim node
    2. Controller
    """

    return LaunchDescription([
        # Start the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen' 
        ),
        #Start the controller node
        Node(
            package='cps_pid_turtle',
            executable='controller',
            name='controller',
            output='screen'
        ),
    ])

