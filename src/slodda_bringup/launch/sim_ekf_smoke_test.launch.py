"""
EKF smoke-test i simulasjon — verifiser at robot_localization laster
og publiserer /odometry/filtered uten kræsj.

Brukes som hardware-readiness sjekk FØR deploy på Pi4.
Ikke for navigasjonstesting — kun for å validere EKF-konfig.

Kjør:
  ros2 launch slodda_bringup sim_ekf_smoke_test.launch.py
  ros2 topic hz /odometry/filtered   # forventet: ~15 Hz
  ros2 topic echo /odometry/filtered --once
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('slodda_bringup')
    ekf_params = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
        ]
    )

    return LaunchDescription([ekf_node])
