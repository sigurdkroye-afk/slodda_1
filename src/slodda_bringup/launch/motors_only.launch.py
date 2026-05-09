import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hw = {'use_sim_time': False}
    motors_cfg = os.path.join(
        get_package_share_directory('slodda_bringup'), 'config', 'motors.yaml')

    motor_driver = Node(
        package='slodda_bringup',
        executable='motor_driver',
        output='screen',
        parameters=[motors_cfg, hw],
    )

    odometry_node = Node(
        package='slodda_bringup',
        executable='odometry_node',
        output='screen',
        parameters=[motors_cfg, hw],
    )

    return LaunchDescription([motor_driver, odometry_node])
