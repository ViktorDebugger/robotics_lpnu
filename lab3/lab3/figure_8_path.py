"""Figure-8 path: two circles, first left (w>0), then right (w<0). Timed motion as circle_path."""
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped

from .diff_drive_math import twist_to_wheel_speeds


class Figure8Path(Node):
    def __init__(self):
        super().__init__('figure_8_path')

        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.3)
        self.declare_parameter("wheel_radius", 0.4)
        self.declare_parameter("wheel_separation", 1.2)
        self.declare_parameter("rate_hz", 20.0)
        # use_sim_time is declared by ROS2; pass -p use_sim_time:=true when in simulation

        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        v = float(self.get_parameter("linear_speed").value)
        w = float(self.get_parameter("angular_speed").value)
        dt = 1.0 / max(float(self.get_parameter("rate_hz").value), 1.0)

        wheel_r = float(self.get_parameter("wheel_radius").value)
        wheel_s = float(self.get_parameter("wheel_separation").value)

        # Slight margin (2%) so the loop closes despite discrete timesteps
        duration = (2.0 * math.pi / max(abs(w), 1e-6)) * 1.02
        wl_left, wr_left = twist_to_wheel_speeds(v, w, wheel_r, wheel_s)
        wl_right, wr_right = twist_to_wheel_speeds(v, -w, wheel_r, wheel_s)

        self.get_logger().info(
            f"Figure-8: v={v:.2f}, w=±{w:.2f}, t={duration:.2f}s per loop | "
            f"Left circle ω: L={wl_left:.2f}, R={wr_left:.2f} | "
            f"Right circle ω: L={wl_right:.2f}, R={wr_right:.2f}"
        )

        # First circle: left (w>0); second: right (w<0)
        for i, angular_z in enumerate([w, -w], start=1):
            side = "left" if angular_z > 0 else "right"
            self.get_logger().info(f"Circle {i}/2 ({side}, angular.z={angular_z:+.2f})")
            msg = TwistStamped()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = v
            msg.twist.angular.z = angular_z

            clock = self.get_clock()
            t_end = clock.now() + Duration(nanoseconds=int(duration * 1e9))
            while clock.now() < t_end:
                msg.header.stamp = clock.now().to_msg()
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(dt)

            time.sleep(0.2)

        self.pub.publish(TwistStamped())
        self.get_logger().info("Figure-8 complete.")


def main(args=None):
    rclpy.init(args=args)
    node = Figure8Path()
    node.destroy_node()
    rclpy.shutdown()
