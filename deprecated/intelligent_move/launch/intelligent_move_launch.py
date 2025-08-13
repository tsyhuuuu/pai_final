#!/usr/bin/env python3
"""
Launch file for Intelligent Move Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='voice',
        description='Operation mode: voice, manual, or auto'
    )
    
    # Create the node
    intelligent_move_node = Node(
        package='intelligent_move',
        executable='intelligent_move_node',
        name='intelligent_move_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode')
        }]
    )
    
    return LaunchDescription([
        mode_arg,
        intelligent_move_node
    ])