import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup     = get_package_share_directory('slodda_bringup')
    pkg_description = get_package_share_directory('slodda_description')

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    ekf_params  = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_bringup, 'config', 'hardware.rviz')
    bt_xml      = os.path.join(pkg_bringup, 'behavior_trees', 'navigate_to_pose_no_spin.xml')

    xacro_file        = os.path.join(pkg_description, 'urdf', 'slodda_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    hw = {'use_sim_time': False}

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz (true/false)')

    # ── Phase 1 (t=0): Hardware drivers ───────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[hw, {'robot_description': robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[hw, {'robot_description': robot_description}]
    )

    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'lidar_link'},
            {'port_name': '/dev/ttyAMA3'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
        ]
    )

    imu_node = Node(
        package='slodda_bringup',
        executable='imu_node',
        output='screen',
        parameters=[hw, {'publish_hz': 20.0, 'frame_id': 'imu_link'}]
    )

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

    # ── Phase 2 (t=4s): Odometry + EKF ──────────────────────────────────────
    # Encoders not wired → ticks always zero → EKF uses IMU yaw-rate only.
    # This gives correct rotation tracking. X/Y position stays at zero until
    # encoders are connected and odom0 is re-enabled in ekf.yaml.
    odometry_node = Node(
        package='slodda_bringup',
        executable='odometry_node',
        output='screen',
        parameters=[hw, {
            'wheel_radius_m': 0.0208,
            'wheel_base_m':   0.256,
            'ticks_per_rev':  663.0,
            'publish_tf':     False,  # EKF publishes odom→base_footprint TF
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, hw]
    )

    # ── Phase 3 (t=6s): Static map→odom TF ───────────────────────────────────
    # Identity transform: robot starts at map origin.
    # TODO: replace with slam_toolbox once scan-processing issue is debugged.
    #   Current issue: slam_toolbox starts but never processes scans.
    #   Suspected cause: tf2 MessageFilter drops scans — needs clean investigation.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── Phase 4 (t=12s): Nav2 servers ─────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[hw, nav2_params],
        remappings=[('cmd_vel', 'cmd_vel')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[hw, nav2_params]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[hw, nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[hw, nav2_params]
    )

    # bt_xml passed explicitly — $(find-pkg-share) in YAML is a launch substitution
    # and is NOT resolved when the YAML is loaded as a --params-file at runtime.
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[hw, nav2_params, {'default_nav_to_pose_bt_xml': bt_xml}]
    )

    # ── Phase 5 (t=22s): Lifecycle manager ────────────────────────────────────
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
            ],
            'bond_timeout': 60.0,
            'bond.heartbeat_period': 2.0,
            'bond.heartbeat_timeout': 30.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 30.0,
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[hw],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        # Phase 1: immediate
        robot_state_publisher,
        joint_state_publisher,
        lidar_node,
        imu_node,
        motor_driver,
        # Phase 2: t=4s — odometry + EKF (IMU yaw fusion)
        TimerAction(period=4.0, actions=[
            odometry_node,
            ekf_node,
        ]),
        # Phase 3: t=6s — static map→odom TF
        TimerAction(period=6.0, actions=[
            map_odom_tf,
        ]),
        # Phase 4a: t=12s — heavy Nav2 servers (each has costmap inside)
        TimerAction(period=12.0, actions=[
            controller_server,
            planner_server,
        ]),
        # Phase 4b: t=18s — lighter servers (no costmap)
        TimerAction(period=18.0, actions=[
            smoother_server,
            behavior_server,
            bt_navigator,
        ]),
        # Phase 5: t=30s — lifecycle manager (extra margin after staggered init)
        TimerAction(period=30.0, actions=[
            lifecycle_manager_navigation,
        ]),
        rviz,
    ])
