#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import SetParametersResult


from f9r_state_interfaces.msg import F9RState

#
# Временный заглушечный импорт ниже нужен только для подсветки типов.
# try:
#     from f9r_state_interfaces.msg import F9RState
# except Exception:
#     # создаём лёгкий shim, чтобы файл был читабелен; при сборке с реальным пакетом — удалить shim
#     from dataclasses import dataclass
#     class F9RState:  # type: ignore
#         yaw_deg: float

def yaw_to_quat(yaw_rad: float) -> Quaternion:
    cy = math.cos(yaw_rad * 0.5); sy = math.sin(yaw_rad * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)

def normalize_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class F9RStateToImu(Node):
    def __init__(self):
        super().__init__('f9r_state_to_imu')
        # реальные топики:
        self.declare_parameter('f9r_state_topic', '/gps_f9r/state_stable')  # F9RState с полем yaw_deg
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('covariance', 0.05)
        self.declare_parameter('yaw_offset_deg', 0.0)

        self.f9r_state_topic = self.get_parameter('f9r_state_topic').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.cov = float(self.get_parameter('covariance').get_parameter_value().double_value)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').get_parameter_value().double_value)

        self.pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.sub = self.create_subscription(F9RState, self.f9r_state_topic, self.cb, 10)
        self.add_on_set_parameters_callback(self.on_params)

        self.get_logger().info(f'[f9r_state_to_imu] {self.f9r_state_topic} → {self.imu_topic} frame={self.imu_frame}')

    def on_params(self, params):
        for p in params:
            if p.name == 'yaw_offset_deg':
                self.yaw_offset_deg = float(p.value)
        return SetParametersResult(successful=True)

    def cb(self, msg: F9RState):
        # ожидается поле yaw_deg: 0 = North, 90 = East (по часовой)
        yaw_north_deg = float(getattr(msg, 'yaw_deg'))
        yaw_north_deg += self.yaw_offset_deg

        # ENU: yaw=0 на Восток
        yaw_enu = math.radians(90.0 - yaw_north_deg)
        yaw_enu = normalize_pi(yaw_enu)

        imu = Imu()
        imu.header = Header()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame
        imu.orientation = yaw_to_quat(yaw_enu)
        imu.orientation_covariance = [
            float(self.cov), 0.0, 0.0,
            0.0, float(self.cov), 0.0,
            0.0, 0.0, float(self.cov)
        ]
        self.pub.publish(imu)

def main():
    rclpy.init()
    node = F9RStateToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()