from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])