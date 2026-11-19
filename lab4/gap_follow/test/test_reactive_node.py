"""Synthetic-scan checks, like the lab's autograder: the node must steer toward the wider
gap around a pillar or along a corridor, and into the exit of a dead end. Each case is
also checked mirrored."""

import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from gap_follow.reactive_node import ReactiveFollowGap

BEAMS = 1080
FOV = 4.7


def raycast(segments, circles, mirror):
    """Scan from the origin, facing +x, against wall segments and round pillars.
    `mirror` flips the world left-to-right."""
    flip = -1.0 if mirror else 1.0
    scan = LaserScan()
    scan.angle_min = -FOV / 2
    scan.angle_increment = FOV / BEAMS
    scan.range_min = 0.0
    scan.range_max = 30.0
    ranges = []
    for i in range(BEAMS):
        a = scan.angle_min + i * scan.angle_increment
        dx, dy = math.cos(a), math.sin(a)
        best = 30.0
        for (x1, y1), (x2, y2) in segments:
            y1, y2 = flip * y1, flip * y2
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = (x1 * ey - y1 * ex) / den
            u = (x1 * dy - y1 * dx) / den
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        for cx, cy, r in circles:
            cy = flip * cy
            b = dx * cx + dy * cy
            c = cx * cx + cy * cy - r * r
            disc = b * b - c
            if disc >= 0 and b - math.sqrt(disc) > 0:
                best = min(best, b - math.sqrt(disc))
        ranges.append(best)
    scan.ranges = ranges
    return scan


def corridor(left, right, length=30.0):
    return [((-2.0, left), (length, left)), ((-2.0, -right), (length, -right))]


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = ReactiveFollowGap()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def steering(node, scan):
    sent = []
    node.drive_publisher.publish = sent.append
    node.lidar_callback(scan)
    return sent[-1].drive.steering_angle


@pytest.mark.parametrize('mirror', [False, True])
def test_pillar_ahead_steers_toward_wider_side(node, mirror):
    # 3 m wide corridor, pillar 3 m ahead sitting 0.4 m right of centre: wider gap on the left.
    scan = raycast(corridor(1.5, 1.5), [(3.0, -0.4, 0.3)], mirror)
    side = -1.0 if mirror else 1.0
    assert side * steering(node, scan) > 0.0


@pytest.mark.parametrize('mirror', [False, True])
def test_near_one_wall_steers_toward_open_side(node, mirror):
    # Car 0.4 m from the right wall of a 2 m corridor: more room on the left.
    scan = raycast(corridor(1.6, 0.4), [], mirror)
    side = -1.0 if mirror else 1.0
    assert side * steering(node, scan) > 0.0


@pytest.mark.parametrize('mirror', [False, True])
def test_dead_end_aims_into_side_exit(node, mirror):
    # Corridor ends 3 m ahead; the way on is a hallway opening to the left at x 1.5 to 3.0.
    walls = [
        ((-2.0, -0.8), (3.0, -0.8)),   # right wall, runs into the end wall
        ((3.0, -0.8), (3.0, 10.0)),    # end wall, continues as the side hallway's far wall
        ((-2.0, 0.8), (1.5, 0.8)),     # left wall up to the opening
        ((1.5, 0.8), (1.5, 10.0)),     # side hallway's near wall
    ]
    scan = raycast(walls, [], mirror)
    side = -1.0 if mirror else 1.0
    assert side * steering(node, scan) > 0.0
