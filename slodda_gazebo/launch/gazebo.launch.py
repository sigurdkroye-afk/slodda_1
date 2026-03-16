import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('slodda_gazebo')
    pkg_description = get_package_share_directory('slodda_description')

    # Install share-rooten – Gazebo trenger denne for å finne meshes
    install_share = os.path.join(pkg_description, '..', '..', '..', '..')

    world_file = os.path.join(pkg_gazebo, 'worlds', 'arena.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'slodda_real.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # Fortell Gazebo hvor ROS-pakker ligger
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_description, '..')
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '--headless-rendering', '--render-engine-server', 'ogre2'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-name', 'slodda',
                '-x', '0', '-y', '0', '-z', '0.1',
                '-topic', 'robot_description'
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/ir_front_left@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/ir_front_center@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/ir_front_right@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ]
        ),
    ])
