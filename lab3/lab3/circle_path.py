"""Circle path using timed motion. Base for figure-8."""
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from .diff_drive_math import twist_to_wheel_speeds


class CirclePath(Node):
    def __init__(self):
        super().__init__('circle_path')

        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.3)
        self.declare_parameter("wheel_radius", 0.4)
        self.declare_parameter("wheel_separation", 1.2)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("odom_topic", "/model/vehicle_blue/odometry")

        odom_topic = self.get_parameter("odom_topic").value
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self.odom_received = False

        self.get_logger().info("Waiting for odometry...")
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        time.sleep(0.5)
        self.get_logger().info("Starting circle path")

        v = float(self.get_parameter("linear_speed").value)
        w = float(self.get_parameter("angular_speed").value)
        dt = 1.0 / max(float(self.get_parameter("rate_hz").value), 1.0)

        wheel_r = float(self.get_parameter("wheel_radius").value)
        wheel_s = float(self.get_parameter("wheel_separation").value)
        wl, wr_val = twist_to_wheel_speeds(v, w, wheel_r, wheel_s)

        duration = 2.0 * math.pi / max(abs(w), 1e-6)
        self.get_logger().info(f"Circle: v={v:.2f}, w={w:.2f}, t={duration:.2f}s | wheel ω: L={wl:.2f}, R={wr_val:.2f}")

        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = v
        msg.twist.angular.z = w

        t_end = time.time() + duration
        while time.time() < t_end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.pub.publish(TwistStamped())
        self.get_logger().info("Circle complete.")

    def _odom_callback(self, msg: Odometry):
        self.odom_received = True


def main(args=None):
    rclpy.init(args=args)
    node = CirclePath()
    node.destroy_node()
    rclpy.shutdown()
