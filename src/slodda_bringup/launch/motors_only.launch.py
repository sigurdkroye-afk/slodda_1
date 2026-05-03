from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hw = {'use_sim_time': False}

    motor_driver = Node(
        package='slodda_bringup',
        executable='motor_driver',
        output='screen',
        parameters=[hw, {
            'wheel_base_m':          0.256,
            'max_wheel_speed_mps':   0.5,
            'min_pwm':               25.0,
            'max_pwm':               95.0,
            'velocity_deadband_mps': 0.01,
            'cmd_vel_timeout_sec':   0.5,
        }]
    )

    odometry_node = Node(
        package='slodda_bringup',
        executable='odometry_node',
        output='screen',
        parameters=[hw, {
            'wheel_radius_m': 0.0208,
            'wheel_base_m':   0.256,
            'ticks_per_rev':  663.0,
            'publish_tf':     True,
        }]
    )

    return LaunchDescription([
        motor_driver,
        odometry_node,
    ])
