import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_bringup = get_package_share_directory('slodda_bringup')
    pkg_gazebo  = get_package_share_directory('slodda_gazebo')
    rviz_config = os.path.join(pkg_bringup, 'config', 'nav2.rviz')

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(pkg_gazebo,  'maps',   'arena_map.yaml')

    sim_time = {'use_sim_time': True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[sim_time, nav2_params]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[sim_time, {'yaml_filename': map_file}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[sim_time, nav2_params],
        remappings=[('cmd_vel', 'cmd_vel')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[sim_time, nav2_params]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[sim_time, nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[sim_time, nav2_params]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[sim_time, nav2_params]
    )

    # Localization starts first — bond_timeout=0 means no service-response timeout
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[sim_time, {
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
        parameters=[sim_time, {
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

    # Localization at 10 s — gives Gazebo time to fully load
    localization_nodes = TimerAction(
        period=10.0,
        actions=[
            map_server,
            amcl,
            lifecycle_manager_localization,
        ]
    )

    # Navigation at 15 s — after localization has time to activate
    navigation_nodes = TimerAction(
        period=15.0,
        actions=[
            controller_server,
            planner_server,
            smoother_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_navigation,
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen'
    )

    object_tracker = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='slodda_bringup',
                executable='object_tracker',
                name='object_tracker',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        localization_nodes,
        navigation_nodes,
        rviz,
        object_tracker,
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='slodda_bringup',
                    executable='yolo_detector',
                    name='yolo_detector',
                    output='screen'
                ),
                Node(
                    package='slodda_bringup',
                    executable='bear_mission',
                    name='bear_mission',
                    output='screen',
                    parameters=[{'goal_x': 1.0, 'goal_y': 0.0}]
                ),
                Node(
                    package='rqt_image_view',
                    executable='rqt_image_view',
                    name='image_view',
                    arguments=['/yolo/image'],
                    output='screen'
                ),
            ]
        ),
    ])
