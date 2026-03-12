"""Dead reckoning: integrate /cmd_vel to estimate pose; publish path to /path_dr.

Compares with Gazebo ground truth (/odom). RViz shows both trajectories.
Reference: https://www.roboticsbook.org/S52_diffdrive_actions.html
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__("dead_reckoning")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped", True)  # True=TwistStamped, False=Twist
        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("path_dr_topic", "/path_dr")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("max_poses", 2000)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        use_stamped = self.get_parameter("cmd_vel_stamped").value
        gt_topic = self.get_parameter("ground_truth_topic").value
        path_topic = self.get_parameter("path_dr_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.max_poses = int(self.get_parameter("max_poses").value)

        if use_stamped:
            self.create_subscription(TwistStamped, cmd_topic, self._cmd_stamped_cb, 10)
        else:
            self.create_subscription(Twist, cmd_topic, self._cmd_twist_cb, 10)
        self.create_subscription(Odometry, gt_topic, self.gt_callback, 10)
        self.pub_path = self.create_publisher(Path, path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        # Publish initial empty path so /path_dr appears
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.path_msg)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_stamp_sec = None
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_theta = 0.0

        self.get_logger().info(
            f"Dead reckoning: cmd_vel={cmd_topic} ({'TwistStamped' if use_stamped else 'Twist'}), path_dr={path_topic}"
        )

        self.create_timer(0.2, self._timer_publish)

    def _cmd_stamped_cb(self, msg: TwistStamped):
        self._integrate(msg.twist.linear.x, msg.twist.angular.z, msg.header.stamp)

    def _cmd_twist_cb(self, msg: Twist):
        stamp = self.get_clock().now().to_msg()
        self._integrate(msg.linear.x, msg.angular.z, stamp)

    def _timer_publish(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.path_msg)

    def _integrate(self, v: float, w: float, stamp) -> None:
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        if self.last_stamp_sec is not None:
            dt = t_sec - self.last_stamp_sec
            if 0.0 < dt < 2.0:
                self.x += v * math.cos(self.theta) * dt
                self.y += v * math.sin(self.theta) * dt
                self.theta += w * dt
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.last_stamp_sec = t_sec

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        qw = math.cos(self.theta / 2.0)
        qz = math.sin(self.theta / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses = self.path_msg.poses[-self.max_poses :]

        self.pub_path.publish(self.path_msg)

    def gt_callback(self, msg: Odometry):
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.gt_theta = math.atan2(siny, cosy)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
