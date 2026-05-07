import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('slodda_bringup')
    rviz_config = os.path.join(pkg_bringup, 'config', 'hardware.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    hardware_panel = Node(
        package='slodda_bringup',
        executable='hardware_panel',
        output='screen'
    )

    return LaunchDescription([rviz, hardware_panel])
