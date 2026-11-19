"""Put the car back on the start line when it crashes.

This version of the simulator stops a crashed car dead (speed and heading snap to 0) and
leaves it against the wall. Newer RoboRacer simulators reset it instead; this node does the
same by sending the start pose on /initialpose, like RViz's 2D Pose Estimate tool.
"""

import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CrashReset(Node):

    def __init__(self):
        super().__init__('crash_reset')
        self.declare_parameter('sx', -12.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)

        self.prev_speed = 0.0
        self.prev_yaw = None
        self.last_reset = None
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.reset_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def odom_callback(self, msg):
        speed = abs(msg.twist.twist.linear.x)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        # A crash: speed drops to exactly 0 in one step from driving speed, or together with
        # the heading snapping. Braking slows the car gradually, so it doesn't count.
        # The reset itself also snaps the heading, so ignore a moment after a reset.
        yaw_jump = self.prev_yaw is not None and abs(
            math.atan2(math.sin(yaw - self.prev_yaw), math.cos(yaw - self.prev_yaw))) > 0.1
        now = self.get_clock().now()
        settling = self.last_reset is not None and (now - self.last_reset).nanoseconds < 0.3e9
        if speed == 0.0 and (self.prev_speed > 0.15 or yaw_jump) and not settling:
            self.reset()
            self.last_reset = now

        self.prev_speed = speed
        self.prev_yaw = yaw

    def reset(self):
        theta = self.get_parameter('stheta').value
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.get_parameter('sx').value
        pose.pose.pose.position.y = self.get_parameter('sy').value
        pose.pose.pose.orientation.z = math.sin(theta / 2)
        pose.pose.pose.orientation.w = math.cos(theta / 2)
        self.reset_publisher.publish(pose)
        self.get_logger().warn('Crash: car put back on the start line')
        self.prev_yaw = None


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # Stop on Ctrl+C and termination even when started where Ctrl+C is ignored by default.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _interrupt)
    node = CrashReset()
    try:
        # Wake every 0.1 s even when nothing arrives, so Ctrl+C is always noticed.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
