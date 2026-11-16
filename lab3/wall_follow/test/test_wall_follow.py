"""Synthetic-scan checks, like the lab's autograder: in a straight corridor the node must
steer away from a wall it is too close to or turned toward."""

import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from wall_follow.wall_follow_node import WallFollow

BEAMS = 1080
FOV = 4.7


def corridor_scan(left, right, yaw_deg=0.0):
    """Scan from a car `left` m from the left wall and `right` m from the right wall,
    turned `yaw_deg` toward the left (positive) or right (negative)."""
    scan = LaserScan()
    scan.angle_min = -FOV / 2
    scan.angle_increment = FOV / BEAMS
    scan.range_min = 0.0
    scan.range_max = 30.0
    ranges = []
    for i in range(BEAMS):
        a = scan.angle_min + i * scan.angle_increment + math.radians(yaw_deg)
        s = math.sin(a)
        r = left / s if s > 1e-6 else right / -s if s < -1e-6 else 30.0
        ranges.append(min(r, 30.0))
    scan.ranges = ranges
    return scan


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = WallFollow()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def steering(node, scan):
    sent = []
    node.drive_publisher.publish = sent.append
    node.prev_time = None
    node.integral = 0.0
    node.prev_error = 0.0
    node.speed = 0.0
    node.scan_callback(scan)
    return sent[-1].drive.steering_angle


def test_too_close_to_left_wall_steers_right(node):
    assert steering(node, corridor_scan(left=0.5, right=1.5)) < 0.0


def test_too_close_to_right_wall_steers_left(node):
    assert steering(node, corridor_scan(left=1.5, right=0.5)) > 0.0


def test_turned_toward_left_wall_steers_right(node):
    assert steering(node, corridor_scan(left=1.0, right=1.0, yaw_deg=20.0)) < 0.0


def test_turned_toward_right_wall_steers_left(node):
    assert steering(node, corridor_scan(left=1.0, right=1.0, yaw_deg=-20.0)) > 0.0


def test_centred_and_straight_drives_straight_and_fast(node):
    sent = []
    node.drive_publisher.publish = sent.append
    node.prev_time = None
    node.speed = 0.0
    node.scan_callback(corridor_scan(left=1.0, right=1.0))
    assert abs(sent[-1].drive.steering_angle) < math.radians(2.0)
    assert sent[-1].drive.speed == node.get_parameter('straight_speed').value


def test_wall_ahead_caps_any_straight_speed(node):
    # Corridor ending in a wall 3 m ahead: even asked for 10 m/s, it must be able to slow
    # for the corner.
    node.set_parameters([rclpy.parameter.Parameter('straight_speed', value=10.0)])
    try:
        scan = corridor_scan(left=1.0, right=1.0)
        centre = round((0.0 - scan.angle_min) / scan.angle_increment)
        scan.ranges = [3.0 if abs(i - centre) < 60 else r for i, r in enumerate(scan.ranges)]
        sent = []
        node.drive_publisher.publish = sent.append
        node.prev_time = None
        node.speed = 0.0
        node.scan_callback(scan)
        # sqrt(corner_speed^2 + 2 * max_decel * (3.0 - corner_margin)) = sqrt(1.5^2 + 2*3*1.5)
        assert sent[-1].drive.speed == pytest.approx(math.sqrt(1.5 ** 2 + 2 * 3.0 * 1.5))
    finally:
        node.set_parameters([rclpy.parameter.Parameter('straight_speed', value=1.5)])
