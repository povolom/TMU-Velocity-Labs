"""Checks of the obstacle_avoidance node's pieces: the distance map, the path shaping, a plan
around an obstacle on a straight piece of track, and following the racing line."""

import math
import os

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from gap_follow import obstacle_avoidance as oa

CENTERLINE = os.path.join(os.path.dirname(__file__), '..', 'config', 'levine_centerline.csv')
LINE = os.path.join(os.path.dirname(__file__), '..', 'config', 'levine_obs_line.csv')


def test_distance_map_measures_to_nearest_point():
    dmap = oa.DistanceMap(np.array([1.0, 3.0]), np.array([0.0, 0.0]),
                          np.array([0.0, 4.0]), np.array([-1.0, 1.0]))
    got = dmap.at(np.array([2.0, 1.0, 3.0]), np.array([0.0, 0.5, 1.0]))
    assert got == pytest.approx([1.0, 0.5, 1.0], abs=oa.RASTER)


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    # The synthetic scans below are taken from the car's own position.
    n = oa.ObstacleAvoidance(parameter_overrides=[
        Parameter('centerline', value=os.path.abspath(CENTERLINE)),
        Parameter('lidar_offset', value=0.0)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def scan_from(x, y, yaw, walls):
    """Ranges of a 1080-beam, 270 degree LiDAR at (x, y, yaw) against wall segments."""
    angles = -2.35 + np.arange(1080) * 4.7 / 1080
    ranges = np.full(1080, 30.0)
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        dx, dy = np.cos(yaw + angles), np.sin(yaw + angles)
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
        hit = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        ranges = np.where(hit, np.minimum(ranges, t), ranges)
    return ranges, angles


def box(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def test_plans_around_an_obstacle_ahead(node):
    # South hallway of Levine (walls at y -0.95 and 0.70), driving east, a box ahead on
    # the right half: the path must pass it on the left, clear of everything.
    x, y, yaw = -5.0, -0.1, 0.0
    walls = [((-15, -0.95), (15, -0.95)), ((-15, 0.70), (15, 0.70))] + box(-2.5, -0.95, -2.0, 0.0)
    ranges, angles = scan_from(x, y, yaw, walls)
    node.speed = 2.0
    steering, speed, (px, py) = node.plan(ranges, angles, x, y, yaw, 2.0, 0.02)
    beside = (px > -2.6) & (px < -1.9)
    assert beside.any(), 'the path should reach past the box'
    assert np.all(py[beside] > 0.0 + oa.HALF_WIDTH)
    assert np.all(py < 0.70 - oa.HALF_WIDTH) and np.all(py > -0.95 + oa.HALF_WIDTH)
    assert speed > 0.0 and math.isfinite(steering)
    # ...and is drivable: no tighter than the car's turning limit.
    heading = np.unwrap(np.arctan2(np.diff(py), np.diff(px)))
    bend = np.abs(np.diff(heading)) / np.hypot(np.diff(px), np.diff(py))[1:]
    assert np.all(bend[5:] <= oa.KAPPA_MAX * 1.3)


@pytest.fixture(scope='module')
def line_node(node):
    n = oa.ObstacleAvoidance(parameter_overrides=[
        Parameter('centerline', value=os.path.abspath(CENTERLINE)),
        Parameter('racing_line', value=os.path.abspath(LINE)),
        Parameter('lidar_offset', value=0.0)])
    yield n
    n.destroy_node()


def on_line_pose(x_near):
    """A pose on the racing line in Levine's south hallway, pointing along it."""
    line = np.loadtxt(LINE, delimiter=',', comments='#')
    south = np.flatnonzero((line[:, 3] < 0.0) & (line[:, 2] > -9.0) & (line[:, 2] < 0.0))
    i = south[np.argmin(np.abs(line[south, 2] - x_near))]
    yaw = math.atan2(line[i + 1, 3] - line[i - 1, 3], line[i + 1, 2] - line[i - 1, 2])
    return line, line[i, 2], line[i, 3], yaw


def off_line(line, px, py):
    t = np.linspace(0.0, len(line) - 1, 20 * len(line))
    lx = np.interp(t, np.arange(len(line)), line[:, 2])
    ly = np.interp(t, np.arange(len(line)), line[:, 3])
    return np.min(np.hypot(px[:, None] - lx[None, :], py[:, None] - ly[None, :]), axis=1)


def test_follows_the_racing_line_when_it_is_clear(line_node):
    # Only the hallway's walls in sight: the path is the racing line itself.
    line, x, y, yaw = on_line_pose(-6.0)
    walls = [((-15, -0.95), (15, -0.95)), ((-15, 0.70), (15, 0.70))]
    ranges, angles = scan_from(x, y, yaw, walls)
    line_node.speed = 2.0
    steering, speed, (px, py) = line_node.plan(ranges, angles, x, y, yaw, 2.0, 0.02)
    assert len(px) > 20 and speed > 0.0
    assert np.all(off_line(line, px, py) < 0.02)


def test_leaves_the_racing_line_round_something_not_on_the_map(line_node):
    # A box on the line 1.5 m ahead: the path goes round it, clear of it, instead.
    line, x, y, yaw = on_line_pose(-6.0)
    bx, by = x + 1.5 * math.cos(yaw), y + 1.5 * math.sin(yaw)
    walls = [((-15, -0.95), (15, -0.95)), ((-15, 0.70), (15, 0.70))] + \
        box(bx - 0.1, by - 0.1, bx + 0.1, by + 0.1)
    ranges, angles = scan_from(x, y, yaw, walls)
    line_node.last_plan = None
    steering, speed, (px, py) = line_node.plan(ranges, angles, x, y, yaw, 2.0, 0.02)
    beside = np.abs(px - bx) < 0.1
    assert beside.any(), 'the path should reach past the box'
    assert np.all(np.abs(py[beside] - by) > 0.1 + oa.HALF_WIDTH)


def test_route_turns_no_tighter_than_the_car_can(node):
    # A straight track along x with its left half blocked ahead: starting at d = 0.3 the
    # route must move right in time, and never turn tighter than KAPPA_MAX on the map.
    rows, cells = 90, 401
    cell_d = -2.0 + np.arange(cells) * oa.CELL_D
    s = np.arange(rows) * oa.CELL_S
    cost = np.zeros((rows, cells))
    cost[40:70, cell_d > -0.1] = np.inf
    d, last = node.route(cost, cell_d, s, np.zeros(rows), np.ones(rows), np.zeros(rows),
                         0.3, 0.0, 0.3, 0.0)
    assert last == rows - 1 - (rows - 1) % oa.ROUTE_STEP
    assert np.all(d[40:70] <= -0.1)
    knots = d[::oa.ROUTE_STEP]
    ds = oa.ROUTE_STEP * oa.CELL_S
    heading = np.arctan2(np.diff(knots), ds)
    turn = np.abs(np.diff(heading)) / np.hypot(ds, np.diff(knots)[1:])
    assert np.all(turn <= oa.KAPPA_MAX * 1.1)
