import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup     = get_package_share_directory('slodda_bringup')
    pkg_description = get_package_share_directory('slodda_description')
    pkg_gazebo      = get_package_share_directory('slodda_gazebo')

    nav2_params = os.path.join(pkg_bringup,  'config', 'nav2_params.yaml')
    ekf_params  = os.path.join(pkg_bringup,  'config', 'ekf.yaml')
    map_default = os.path.join(pkg_gazebo,   'maps',   'arena_map.yaml')
    rviz_config = os.path.join(pkg_bringup,  'config', 'nav2.rviz')

    xacro_file       = os.path.join(pkg_description, 'urdf', 'slodda_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    hw = {'use_sim_time': False}

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map', default_value=map_default,
        description='Full path to map yaml file')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz for debugging (true/false)')

    # ── Robot description ─────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[hw, {'robot_description': robot_description}]
    )

    # ── Hardware drivers ──────────────────────────────────────────────────────

    # LD06 LiDAR — adjust port_name if the device appears elsewhere
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'lidar_link'},      # matches URDF — no extra TF needed
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
        ]
    )

    # BNO085 IMU — pip install adafruit-circuitpython-bno08x adafruit-blinka on Pi
    imu_node = Node(
        package='slodda_bringup',
        executable='imu_node',
        output='screen',
        parameters=[hw, {
            'publish_hz': 50.0,
            'frame_id':   'imu_link',
        }]
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

    odometry_node = Node(
        package='slodda_bringup',
        executable='odometry_node',
        output='screen',
        parameters=[hw, {
            'wheel_radius_m': 0.0208,
            'wheel_base_m':   0.256,
            'ticks_per_rev':  663.0,
            'publish_tf':     False,  # EKF publishes odom → base_footprint TF
        }]
    )

    # ── EKF — fuses /odom + /imu/data ────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, hw]
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[hw, nav2_params,
                    {'yaml_filename': LaunchConfiguration('map')}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[hw, nav2_params]
    )

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

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 0.0,
        }]
    )

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
            'bond_timeout': 0.0,
        }]
    )

    # ── RViz (optional, enable with rviz:=true) ───────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[hw],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        rviz_arg,
        robot_state_publisher,
        lidar_node,
        imu_node,
        motor_driver,
        odometry_node,
        ekf_node,
        map_server,
        amcl,
        controller_server,
        planner_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        rviz,
    ])
