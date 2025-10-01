from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    params = [ThisLaunchFileDir().replace('launch', 'config') + '/dual_ekf_navsat_params.yaml']

    return LaunchDescription([
        # 1) Yaw из F9RState -> ENU IMU (курс), можно быстро крутить yaw_offset_deg
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
                # Быстрый софт-сдвиг курса, если есть постоянный оффсет
                'yaw_offset_deg': 0.0
            }]
        ),

        # 2) Статический TF base_link -> gps_antenna (плечо антенны). Сейчас антенна в центре => (0,0,0)
        Node(
            package='gnss_waypoints_nav',
            executable='gps_static_tf_pub',
            name='gps_static_tf_pub',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'gps_frame':  'gps_antenna',
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0
            }]
        ),

        # 3) EKF-odom
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
             output='screen', parameters=params,
             remappings=[('/odometry/filtered', '/odometry/filtered')]),

        # 4) EKF-map
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_map',
             output='screen', parameters=params,
             remappings=[('/odometry/filtered', '/odometry/global')]),

        # 5) navsat_transform (GNSS -> /odometry/gps) с реальными топиками
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
             output='screen', parameters=params,
             remappings=[
                 ('/imu/data', '/imu/data'),
                 ('/gps/fix', '/ublox_gps_node/fix'),
                 ('/odometry/filtered', '/odometry/filtered'),
                 ('/odometry/gps', '/odometry/gps')
             ])
    ])