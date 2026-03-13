"""
Launch file for Hello Robot test
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slodda_bringup',
            executable='hello_robot',
            name='hello_robot',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
