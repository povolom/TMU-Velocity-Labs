#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class Talker(Node):

    def __init__(self):
        super().__init__('talker')

        # Create the two required ROS parameters.
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

        # Create a publisher that publishes AckermannDriveStamped
        # messages on the "drive" topic.
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10
        )

    def publish_drive(self):
        # Get the current parameter values.
        v = self.get_parameter('v').value
        d = self.get_parameter('d').value

        # Create the message.
        msg = AckermannDriveStamped()

        # Put the parameters into the required fields.
        msg.drive.speed = float(v)
        msg.drive.steering_angle = float(d)

        # Publish to /drive.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    talker = Talker()

    # Publish continuously, as fast as this loop can run.
    while rclpy.ok():
        talker.publish_drive()

        # Allows ROS to process things such as parameter changes.
        rclpy.spin_once(talker, timeout_sec=0)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
