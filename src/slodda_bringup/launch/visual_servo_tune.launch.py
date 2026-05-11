"""
visual_servo_tune.launch.py — Minimal hardware launch for VISUAL_SERVO-tuning.

Starter:
  - Kjerneinfrastruktur (TF, LiDAR, IMU, motor, odometri, EKF)
  - arm_controller (for fysisk ARM-test)
  - camera_node + yolo_detector (YOLO-pipeline)
  - bear_mission med alle servo-params som LaunchArguments
  - RViz med hardware.rviz-config (kamera- og YOLO-overlay)

Ingen Nav2 — roboten er forhåndsposisjonert foran bjørnen.
Trigger: ros2 topic pub /bear_mission/start std_msgs/msg/Bool '{data: true}'
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup     = get_package_share_directory('slodda_bringup')
    pkg_description = get_package_share_directory('slodda_description')

    rviz_config       = os.path.join(pkg_bringup, 'config', 'hardware.rviz')
    ekf_params        = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    xacro_file        = os.path.join(pkg_description, 'urdf', 'slodda_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    hw = {'use_sim_time': False}

    # ── Servo-tuning params ───────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('target_cx_offset_norm', default_value='-0.15',
                              description='Normalisert horisontal offset (venstre=-1, høyre=+1)'),
        DeclareLaunchArgument('k_p_yaw',               default_value='1.5',
                              description='P-gain yaw (rad/s per normalisert pikselenhet)'),
        DeclareLaunchArgument('max_lin_servo',         default_value='0.05',
                              description='Maks lineær hastighet i VISUAL_SERVO (m/s)'),
        DeclareLaunchArgument('max_ang_servo',         default_value='0.5',
                              description='Maks vinkelhastig het i VISUAL_SERVO (rad/s)'),
        DeclareLaunchArgument('heading_tolerance',     default_value='0.10',
                              description='Normalisert pikseltoleranse for full hastighet'),
        DeclareLaunchArgument('lost_timeout_s',        default_value='1.0',
                              description='Sekunder uten deteksjon før sveip-søk starter'),
        DeclareLaunchArgument('grab_timeout_s',        default_value='30.0',
                              description='Total VISUAL_SERVO-timeout før retur til SEARCH'),
    ]

    # ── Infrastruktur ─────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[hw, {'robot_description': robot_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[hw, {'robot_description': robot_description}],
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[hw],
    )

    imu_node = Node(
        package='slodda_bringup',
        executable='imu_node',
        output='screen',
        parameters=[hw, {'publish_hz': 50.0, 'frame_id': 'imu_link', 'i2c_address': 0x28}],
    )

    motor_driver = Node(
        package='slodda_bringup',
        executable='motor_driver',
        output='screen',
        parameters=[hw, {
            'wheel_base_m':          0.2316,
            'max_wheel_speed_mps':   0.5,
            'min_pwm':               25.0,
            'max_pwm':               95.0,
            'velocity_deadband_mps': 0.005,
            'cmd_vel_timeout_sec':   0.5,
            'left_trim':             1.0,
        }],
    )

    odometry_node = Node(
        package='slodda_bringup',
        executable='odometry_node',
        output='screen',
        parameters=[hw, {
            'wheel_radius_m': 0.01021,
            'wheel_base_m':   0.2316,
            'ticks_per_rev':  663.0,
            'publish_tf':     False,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, hw],
    )

    arm_controller = Node(
        package='slodda_bringup',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
    )

    # ── Kamera + YOLO ─────────────────────────────────────────────────────────
    camera_node = Node(
        package='slodda_bringup',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[hw],
    )

    yolo_detector = Node(
        package='slodda_bringup',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[hw],
    )

    # ── Bear mission med tune-params ──────────────────────────────────────────
    bear_mission = Node(
        package='slodda_bringup',
        executable='bear_mission',
        name='bear_mission',
        output='screen',
        parameters=[hw, {
            'target_cx_offset_norm': LaunchConfiguration('target_cx_offset_norm'),
            'k_p_yaw':               LaunchConfiguration('k_p_yaw'),
            'max_lin_servo':         LaunchConfiguration('max_lin_servo'),
            'max_ang_servo':         LaunchConfiguration('max_ang_servo'),
            'heading_tolerance':     LaunchConfiguration('heading_tolerance'),
            'lost_timeout_s':        LaunchConfiguration('lost_timeout_s'),
            'grab_timeout_s':        LaunchConfiguration('grab_timeout_s'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[hw],
        output='screen',
    )

    return LaunchDescription([
        *args,
        # t=0: infrastruktur
        robot_state_publisher,
        joint_state_publisher,
        static_map_to_odom,
        imu_node,
        motor_driver,
        arm_controller,
        # t=4s: odometri + EKF
        TimerAction(period=4.0, actions=[odometry_node, ekf_node]),
        # t=6s: kamera + YOLO + mission + RViz
        TimerAction(period=6.0, actions=[
            camera_node,
            yolo_detector,
            bear_mission,
            rviz,
        ]),
    ])
