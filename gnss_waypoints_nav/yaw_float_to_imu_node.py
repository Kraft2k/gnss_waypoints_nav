#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import Float64 as YawMsg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import SetParametersResult

def yaw_to_quat(yaw_rad: float) -> Quaternion:
    cy = math.cos(yaw_rad * 0.5); sy = math.sin(yaw_rad * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)

def normalize_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class YawFloatToImu(Node):
    def __init__(self):
        super().__init__('yaw_float_to_imu')
        self.declare_parameter('yaw_topic', '/gps_f9r/yaw_deg')  # Float64, 0=N, 90=E
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('covariance', 0.05)
        self.declare_parameter('yaw_offset_deg', 0.0)

        self.yaw_topic = self.get_parameter('yaw_topic').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.cov = float(self.get_parameter('covariance').get_parameter_value().double_value)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').get_parameter_value().double_value)

        self.pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.sub = self.create_subscription(YawMsg, self.yaw_topic, self.cb, 10)
        self.add_on_set_parameters_callback(self.on_params)

        self.get_logger().info(f'[yaw_float_to_imu] {self.yaw_topic} → {self.imu_topic} frame={self.imu_frame}')

    def on_params(self, params):
        for p in params:
            if p.name == 'yaw_offset_deg':
                self.yaw_offset_deg = float(p.value)
        return SetParametersResult(successful=True)

    def cb(self, msg: YawMsg):
        yaw_north_deg = float(msg.data) + self.yaw_offset_deg
        yaw_enu = math.radians(90.0 - yaw_north_deg)
        yaw_enu = normalize_pi(yaw_enu)

        imu = Imu()
        imu.header = Header()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame
        imu.orientation = yaw_to_quat(yaw_enu)
        imu.orientation_covariance = [self.cov,0,0, 0,self.cov,0, 0,0,self.cov]
        imu.orientation_covariance = [
            float(self.cov), 0.0, 0.0,
            0.0, float(self.cov), 0.0,
            0.0, 0.0, float(self.cov)
        ]
        self.pub.publish(imu)

def main():
    rclpy.init()
    node = YawFloatToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()