import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('slodda_bringup')
    hw = {'use_sim_time': False}

    # ── Launch arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('goal_x',                default_value='1.0'),
        DeclareLaunchArgument('goal_y',                default_value='0.0'),
        DeclareLaunchArgument('target_cx_offset_norm', default_value='-0.15'),
        DeclareLaunchArgument('k_p_yaw',               default_value='1.5'),
        DeclareLaunchArgument('max_lin_servo',         default_value='0.05'),
        DeclareLaunchArgument('max_ang_servo',         default_value='0.5'),
        DeclareLaunchArgument('heading_tolerance',     default_value='0.10'),
        DeclareLaunchArgument('lost_timeout_s',        default_value='1.0'),
        DeclareLaunchArgument('grab_timeout_s',        default_value='30.0'),
        DeclareLaunchArgument('replay_lin_speed',      default_value='0.10'),
        DeclareLaunchArgument('replay_xy_tol',         default_value='0.15'),
        DeclareLaunchArgument('replay_yaw_tol',        default_value='0.25'),
    ]

    # ── Full hardware stack (includes arm_controller + Nav2) ─────────────────
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'hardware.launch.py')
        )
    )

    # ── Mission nodes at t=35s (after lifecycle manager at t=30s) ────────────
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

    bear_mission = Node(
        package='slodda_bringup',
        executable='bear_mission',
        name='bear_mission',
        output='screen',
        parameters=[hw, {
            'goal_x':                LaunchConfiguration('goal_x'),
            'goal_y':                LaunchConfiguration('goal_y'),
            'target_cx_offset_norm': LaunchConfiguration('target_cx_offset_norm'),
            'k_p_yaw':               LaunchConfiguration('k_p_yaw'),
            'max_lin_servo':         LaunchConfiguration('max_lin_servo'),
            'max_ang_servo':         LaunchConfiguration('max_ang_servo'),
            'heading_tolerance':     LaunchConfiguration('heading_tolerance'),
            'lost_timeout_s':        LaunchConfiguration('lost_timeout_s'),
            'grab_timeout_s':        LaunchConfiguration('grab_timeout_s'),
            'replay_lin_speed':      LaunchConfiguration('replay_lin_speed'),
            'replay_xy_tol':         LaunchConfiguration('replay_xy_tol'),
            'replay_yaw_tol':        LaunchConfiguration('replay_yaw_tol'),
        }],
    )

    return LaunchDescription([
        *args,
        hardware,
        TimerAction(period=35.0, actions=[
            camera_node,
            yolo_detector,
            bear_mission,
        ]),
    ])
