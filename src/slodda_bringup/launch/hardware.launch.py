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
        parameters=[hw, {'publish_hz': 5.0, 'frame_id': 'imu_link'}]
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

    # Static map→odom identity transform.
    # This gives Nav2 a valid map frame immediately without AMCL or SLAM.
    # The robot starts at map origin (0,0,0). Odometry drift will accumulate
    # over time but navigation works correctly for demos and testing.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── Phase 2 (t=4s): Odometry + EKF ───────────────────────────────────────
    odometry_node = Node(
        package='slodda_bringup',
        executable='odometry_node',
        output='screen',
        parameters=[hw, {
            'wheel_radius_m': 0.0208,
            'wheel_base_m':   0.256,
            'ticks_per_rev':  663.0,
            'publish_tf':     False,
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, hw]
    )

    # ── Phase 3 (t=10s): Nav2 servers ─────────────────────────────────────────
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

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[hw, nav2_params]
    )

    # ── Phase 4 (t=20s): Lifecycle manager ────────────────────────────────────
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
        lidar_node,
        imu_node,
        motor_driver,
        map_odom_tf,
        # Phase 2: t=4s
        TimerAction(period=4.0, actions=[
            odometry_node,
            ekf_node,
        ]),
        # Phase 3: t=10s
        TimerAction(period=10.0, actions=[
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
        ]),
        # Phase 4: t=20s
        TimerAction(period=20.0, actions=[
            lifecycle_manager_navigation,
        ]),
        # Phase 5: t=35s — after Nav2 is fully active
        TimerAction(period=35.0, actions=[
            Node(
                package='slodda_bringup',
                executable='camera_node',
                output='screen',
                parameters=[hw],
            ),
        ]),
        rviz,
    ])
