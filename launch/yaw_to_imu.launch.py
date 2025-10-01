from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_waypoints_nav',
            executable='yaw_float_to_imu',
            name='yaw_float_to_imu',
            output='screen',
            parameters=[{
                'yaw_topic': '/gps_f9r/yaw_deg',
                'imu_topic': '/imu/data',
                'imu_frame': 'base_link',
                'covariance': 0.05,
                'yaw_offset_deg': 0.0
            }]
        )
    ])