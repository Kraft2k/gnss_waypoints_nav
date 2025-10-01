#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def deg2rad(d): return d * math.pi / 180.0

def rpy_to_quat(roll, pitch, yaw):
    # roll(X), pitch(Y), yaw(Z)
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
    return (cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy)

class GpsStaticTFBroadcaster(Node):
    """
    Публикует ОДИН статический трансформ base_frame -> gps_frame.
    Используй для задания плеча антенны (lever arm). Ориентацию здесь трогать не нужно,
    курс робота даёт f9r_state_to_imu через yaw (динамика).
    """
    def __init__(self):
        super().__init__('gps_static_tf_pub')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gps_frame', 'gps_antenna')
        self.declare_parameter('x', 0.0)   # метры
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        # Оставляем R/P/Y на случай изменения монтажного угла антенны. По умолчанию 0.
        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('yaw_deg', 0.0)

        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        gps_frame  = self.get_parameter('gps_frame').get_parameter_value().string_value
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        z = float(self.get_parameter('z').value)
        rd = float(self.get_parameter('roll_deg').value)
        pd = float(self.get_parameter('pitch_deg').value)
        yd = float(self.get_parameter('yaw_deg').value)

        roll, pitch, yaw = deg2rad(rd), deg2rad(pd), deg2rad(yd)
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = base_frame
        tf.child_frame_id = gps_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(tf)
        self.get_logger().info(
            f"[gps_static_tf_pub] {base_frame} -> {gps_frame}  "
            f"t=({x:.3f},{y:.3f},{z:.3f}) m, rpy=({rd:.1f},{pd:.1f},{yd:.1f}) deg"
        )

def main():
    rclpy.init()
    node = GpsStaticTFBroadcaster()
    # Статический TF отправлен один раз; оставляем ноду жить, чтобы её можно было убить штатно.
    rclpy.spin(node)
    rclpy.shutdown()