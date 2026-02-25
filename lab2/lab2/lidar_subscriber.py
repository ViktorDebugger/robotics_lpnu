import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10,
        )

        self.processed_pub = self.create_publisher(
            Float32MultiArray,
            '/lidar_processed',
            10,
        )

        self.get_logger().info('LiDAR Subscriber started - listening to /lidar')

    def lidar_callback(self, msg: LaserScan):
        """Called automatically whenever a new LiDAR message arrives"""

        def is_valid(r: float) -> bool:
            return msg.range_min < r < msg.range_max

        valid_ranges = [r for r in msg.ranges if is_valid(r)]

        if not valid_ranges:
            self.get_logger().info('No valid LiDAR data')
            return

        min_distance = min(valid_ranges)
        max_distance = max(valid_ranges)
        avg_distance = sum(valid_ranges) / len(valid_ranges)

        self.get_logger().info(
            f'LiDAR: min={min_distance:.2f}m, '
            f'max={max_distance:.2f}m, '
            f'avg={avg_distance:.2f}m, '
            f'points={len(valid_ranges)}/{len(msg.ranges)}'
        )

        n = len(msg.ranges)
        left_end = n // 3
        right_start = 2 * n // 3

        left_raw = msg.ranges[0:left_end]
        front_raw = msg.ranges[left_end:right_start]
        right_raw = msg.ranges[right_start:n]

        left = [r for r in left_raw if is_valid(r)]
        front = [r for r in front_raw if is_valid(r)]
        right = [r for r in right_raw if is_valid(r)]

        def stats(section, fallback):
            if not section:
                return fallback, fallback
            m = min(section)
            a = sum(section) / len(section)
            return m, a

        fallback = msg.range_max if msg.range_max > 0.0 else 0.0
        min_left, avg_left = stats(left, fallback)
        min_front, avg_front = stats(front, fallback)
        min_right, avg_right = stats(right, fallback)

        if front and min_front < 1.0:
            self.get_logger().warn(
                f'Front obstacle detected at {min_front:.2f}m!'
            )

        self.get_logger().info(
            'Free space (min distances) - '
            f'left={min_left:.2f}m, '
            f'front={min_front:.2f}m, '
            f'right={min_right:.2f}m'
        )

        msg_out = Float32MultiArray()
        msg_out.data = [
            float(min_left),
            float(min_front),
            float(min_right),
            float(avg_left),
            float(avg_front),
            float(avg_right),
        ]
        self.processed_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()