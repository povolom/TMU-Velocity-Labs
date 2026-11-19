"""Publish /clock from a monotonic clock.

WSL's system clock can jump by minutes when Ubuntu's time sync fights Windows' time sync,
which freezes ROS timers until the clock catches up. Nodes that use sim time follow this
clock instead, so the simulator keeps running through the jumps.
"""

import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main(args=None):
    rclpy.init(args=args)
    node = Node('steady_clock')
    publisher = node.create_publisher(Clock, '/clock', 10)

    start_wall = time.time()
    start_steady = time.monotonic()
    msg = Clock()
    try:
        while rclpy.ok():
            now = start_wall + (time.monotonic() - start_steady)
            msg.clock.sec = int(now)
            msg.clock.nanosec = int((now - int(now)) * 1e9)
            publisher.publish(msg)
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
