"""Obstacle avoidance using Artificial Potential Fields."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_x", 3.0)
        self.declare_parameter("goal_y", 3.0)
        
        self.k_att = 1
        self.k_rep = 0.01
        self.safe_dist = 0.4

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        cmd_topic = self.get_parameter("cmd_vel_topic").value

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.pub_cmd = self.create_publisher(TwistStamped, cmd_topic, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def scan_callback(self, msg: LaserScan):
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def control_loop(self):
        if not self.scan_ranges:
            return

        dist_to_goal = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        self.get_logger().info(str(dist_to_goal))

        if dist_to_goal < 0.001:
            self.get_logger().info("Goal reached!", throttle_duration_sec=2.0)
            self.pub_cmd.publish(TwistStamped())
            return

        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_to_goal_local = angle_to_goal - self.theta
        
        while angle_to_goal_local > math.pi: angle_to_goal_local -= 2.0 * math.pi
        while angle_to_goal_local < -math.pi: angle_to_goal_local += 2.0 * math.pi

        F_x = self.k_att * math.cos(angle_to_goal_local)
        F_y = self.k_att * math.sin(angle_to_goal_local)

        for i, r in enumerate(self.scan_ranges):
            if r < 0.05 or math.isinf(r) or math.isnan(r):
                continue
            if r < self.safe_dist:
                angle = self.scan_angle_min + i * self.scan_angle_increment
                rep_mag = self.k_rep * (1.0 / r - 1.0 / self.safe_dist) * (1.0 / (r**2))
                F_x -= rep_mag * math.cos(angle)
                F_y -= rep_mag * math.sin(angle)

        v = F_x
        w = math.atan2(F_y, F_x) * 2.0

        v = max(0.0, min(0.2, v))
        w = max(-1.0, min(1.0, w))

        if v < 0.02 and abs(w) < 0.1 and dist_to_goal > 0.5:
            w = 0.5

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()