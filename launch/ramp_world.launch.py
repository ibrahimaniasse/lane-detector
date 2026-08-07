"""
ramp_world.launch.py
--------------------
ROS 2 Python launch file for the lane-detector package.

Starts the full stack with one command:
  ros2 launch lane_detector ramp_world.launch.py

What it does:
  1. Launches Gazebo Harmonic (gz-sim) with ramp_world.world
  2. Publishes the robot URDF to /robot_description
  3. Spawns the TurtleBot3 Burger into Gazebo via ros_gz_sim
  4. Starts ros_gz_bridge to bridge gz ↔ ROS 2 topics:
       /cmd_vel, /odom, /tf, /joint_states, /camera/image,
       /camera/camera_info, /imu, /scan
  5. Launches the 3 lane-detector nodes:
       detect_lane, control_lane, control_blind
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, AppendEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lane_detector')

    # ---- Arguments ----
    x_pos = LaunchConfiguration('x_pos', default='1.05')
    y_pos = LaunchConfiguration('y_pos', default='1.3')
    z_pos = LaunchConfiguration('z_pos', default='0.01')
    yaw   = LaunchConfiguration('yaw',   default='1.5708')

    # ---- Robot description (xacro → URDF) ----
    urdf_file = os.path.join(pkg, 'urdf',
                             'turtlebot3_burger_high_traction.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # ---- Gazebo Harmonic ----
    world_file = os.path.join(pkg, 'worlds', 'ramp_world.world')
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ---- Robot state publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    # ---- Spawn robot ----
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-topic', 'robot_description',
            '-x', x_pos, '-y', y_pos, '-z', z_pos, '-Y', yaw,
        ],
        output='screen',
    )

    # ---- ros_gz_bridge: gz topics ↔ ROS 2 ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ---- Lane detector nodes ----
    # Delayed slightly to let Gazebo and the bridge settle first
    detect_lane = Node(
        package='lane_detector',
        executable='detect_lane',
        name='detect_lane',
        parameters=[{
            'image_topic': '/camera/image',
            'use_sim_time': True,
        }],
        output='screen',
    )

    control_lane = Node(
        package='lane_detector',
        executable='control_lane',
        name='control_lane',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    control_blind = Node(
        package='lane_detector',
        executable='control_blind',
        name='control_blind',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Delay the lane nodes by 5 s to give Gazebo time to start
    lane_nodes = TimerAction(
        period=5.0,
        actions=[detect_lane, control_lane, control_blind],
    )

    models_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg, 'models'))

    return LaunchDescription([
        models_path,
        DeclareLaunchArgument('x_pos', default_value='1.05'),
        DeclareLaunchArgument('y_pos', default_value='1.3'),
        DeclareLaunchArgument('z_pos', default_value='0.01'),
        DeclareLaunchArgument('yaw',   default_value='1.5708'),
        gazebo,
        rsp,
        spawn,
        bridge,
        lane_nodes,
    ])
