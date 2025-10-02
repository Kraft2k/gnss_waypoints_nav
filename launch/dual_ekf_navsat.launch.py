#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('gnss_waypoints_nav'), 'config', 'dual_ekf_navsat_params.yaml'
    ])

    return LaunchDescription([
        # 1) F9R yaw -> IMU (публикует /imu/data)
        Node(
            package='gnss_waypoints_nav',
            executable='f9r_state_to_imu',
            name='f9r_state_to_imu',
            output='screen',
            parameters=[{
                'f9r_state_topic': '/gps_f9r/state_stable',
                'imu_topic': '/imu/data',
                'imu_frame': 'base_link',
                'covariance': 0.05,
                'yaw_offset_deg': 0.0
            }]
        ),

        # 2) Статический TF. ВАРИАНТ 1: у тебя сейчас frame_id у GPS = "gps"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_tf_pub',
            arguments=['0','0','0','0','0','0','base_link','gps'],   # <-- child = gps
            output='screen'
        ),
        # Если позже поменяешь frame_id драйвера на "gps_antenna", здесь тоже поменяешь 'gps' -> 'gps_antenna'.

        # 3) EKF(odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[params_file],
            remappings=[('/odometry/filtered', '/odometry/filtered')]
        ),

        # 4) EKF(map)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[params_file],
            remappings=[('/odometry/filtered', '/odometry/global')]
        ),

        # 5) navsat_transform с ПРАВИЛЬНЫМИ ремапами
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                params_file,
                {'base_link_frame': 'base_link'}   # ← явная установка при запуске
            ],
            remappings=[
                ('/imu', '/imu/data'),
                ('/gps/fix', '/ublox_gps_node/fix'),
                ('/odometry/filtered', '/odometry/filtered'),
                ('/odometry/gps', '/odometry/gps')
            ]
        ),
    ])