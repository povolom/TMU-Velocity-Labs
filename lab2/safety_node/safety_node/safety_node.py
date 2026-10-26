import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('ttc_threshold', 1.0)

        self.speed = 0.0
        self.braking = False

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        threshold = self.get_parameter('ttc_threshold').value

        min_ttc = math.inf

        for i, distance in enumerate(scan_msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < scan_msg.range_min:
                continue

            if distance > scan_msg.range_max:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            closing_speed = self.speed * math.cos(angle)

            if closing_speed <= 0.0:
                continue

            ttc = distance / closing_speed

            if ttc < min_ttc:
                min_ttc = ttc

        if min_ttc < threshold:
            brake_msg = AckermannDriveStamped()

            brake_msg.header.stamp = self.get_clock().now().to_msg()
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0

            self.drive_publisher.publish(brake_msg)

            if not self.braking:
                self.get_logger().warn(
                    f'EMERGENCY BRAKE: iTTC = {min_ttc:.3f} s'
                )

            self.braking = True

        else:
            self.braking = False


def main(args=None):
    rclpy.init(args=args)

    node = SafetyNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
