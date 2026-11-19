"""Lab 4 on maps with obstacles: know the track, see what's on it, drive a path around it.

The overall shape borrows from two open racing stacks: ForzaETH's race_stack (a known
track, the car located on it, obstacles judged in track coordinates) and the UNICORN racing
stack (an avoidance path that respects the car's turning radius and starts the way the car
is pointing). The pieces here are written for this lab:

  - Track: the levine centerline and its width on each side (config/levine_centerline.csv).
  - Where the car is: the simulator's odometry, moved ahead by the measured lag.
  - What's in the way: every LiDAR point (and the shadow behind it, until it's seen), as a
    distance map.
  - Route: a grid along the track ahead (distance along it x offset from the centerline).
    The cheapest drivable route through it (room around the car, little steering, close to
    the last plan, no turn tighter than the car can make) comes from dynamic programming.
  - Steering: follow the route from the front axle, then check the command by simulating
    the car a moment ahead; speed from the route's bends, the room in its gaps and the
    braking distance to its end.
"""

import os
import signal

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 0.33          # m
LATENCY = 0.025           # s from a scan to its command taking effect, to start with...
LATENCY_MIN, LATENCY_MAX = 0.01, 0.15   # ...then measured, within these
PHYSICS_STEP = 0.01       # s between the simulator's physics steps
MAX_STEERING = 0.4189     # rad
LATERAL_ACCEL = 6.0       # m/s^2 of grip in a turn
BRAKE_DECEL = 5.0         # m/s^2 of braking
ACCEL = 5.0               # m/s^2 the speed profile may gain

# The car's footprint for planning: the simulator's collision box is 0.33 m x 0.31 m,
# covered by three discs along its length. Plus the room to keep: the LiDAR is ~1 cm noisy.
DISCS = (-0.11, 0.0, 0.11)  # m along the car
DISC_RADIUS = 0.164       # m
HALF_WIDTH = 0.155        # m
MARGIN = 0.04             # m
EDGE_SLACK = 0.3          # m allowed past the track's measured edges
HARD_MARGIN = 0.01        # m: closer than this to touching is never planned...
START_SLACK = 0.04        # ...except up to this much closer right by the car...
START_EASE = 0.5          # ...easing back to HARD_MARGIN over this many m of route
MARGIN_WEIGHT = 30.0      # cost per m cut into MARGIN (on top of NEAR_WEIGHT's)
RASTER = 0.02             # m: squares of the distance map
HEADING_BLEND = 0.6       # m over which the footprint turns from the car's heading
SHADOW = 0.6              # m behind each LiDAR hit treated as unknown...
SHADOW_COST = 5.0         # ...costing this per grid row a route spends in it...
UNSEEN_SPEED = 1.0        # ...m/s the car may still be doing when it gets there

# Planning grid.
CELL_S = 0.05             # m
CELL_D = 0.01             # m
HORIZON = 4.0             # m ahead, plus braking distance...
HORIZON_MAX = 12.0        # m ...up to this
# Most change in d per m along the track (the route's angle to the centerline, ~50 deg at
# 1.2). The turning limit (KAPPA_MAX) is what keeps routes drivable; this only bounds the
# search, and must allow cutting across the track's frame in tight corners.
SLOPE = 1.2

# Route search (route).
ROUTE_STEP = 3            # grid rows per step, at least...
ROUTE_STEPS_MAX = 36      # ...and more on long routes, to keep to this many steps
ROUTE_CELLS = 2           # grid cells across the track per route position
KAPPA_MAX = 1.15          # 1/m: tightest turn planned (the LiDAR point can do ~1.27)
BEND_COST = 0.5           # per (1/m)^2 of the route's turning, per m
MAX_SLOPE_CHANGE = 4      # cells of slope change per step searched (sharper is never drivable)
CENTER_WEIGHT = 0.1       # per m^2 off-center (without a racing line)...
LINE_WEIGHT = 200.0       # ...per m^2 off the racing line (with one: routes round
                          # an obstacle, then straight back to the line)
NEAR_WEIGHT = 10.0        # for being within NEAR of something, per m inside it
NEAR = 0.15               # m
STICK_WEIGHT = 10.0       # per m^2 away from the previous plan

# Path tracking (track_path).
GAP_GAIN = 3.0            # how hard to close a sideways gap to the path
CONTROL_PREVIEW = 0.05    # s further ahead (past the lag) to read the path's bend...
BEND_CHORD = 0.3          # ...over this length of path (m)

# Checking each steering command (check_command).
CHECK_TIME = 0.8          # s simulated ahead, at least (longer if needed to brake)
CHECK_DT = 0.05           # s per simulated step
HOLD = 0.3                # s the first steering angle is held
STEER_RATE = 3.2          # rad/s the steering turns at
CHECK_MARGIN = 0.02       # m the simulated footprint must keep
SPEED_GAIN_LIMIT = 1.5    # m/s: the car can't be faster than this over its speed now
KEEP_GOING = 1.0          # m the car must be able to drive on past HOLD
COMMITTED = 0.1           # s at the start of each simulated run that can't be changed
CHECK_PATH_WEIGHT = 4.0   # cost per m^2 an alternative ends away from the path
CHECK_STICK_WEIGHT = 2.0  # cost per rad^2 an alternative differs from the last command

# Steering smoothness.
STEER_SMOOTHING = 0.04    # s: time constant of the steering filter
STEER_SLEW = 2.5          # rad/s the steering command may change at most (car: ~3.2)

# Speed follows the precision the route needs: anywhere on it, at most TIGHT_SPEED plus
# SPEED_PER_ROOM for each m of room to spare around the car there, braking in time for the
# tight spots ahead. A slot with a few cm to spare is taken at walking pace (where the lag
# and tracking error matter); open hallway lets the car run at max_speed.
TIGHT_SPEED = 0.8         # m/s
SPEED_PER_ROOM = 8.0      # m/s per m of room
CREEP_SPEED = 0.5         # m/s when cutting into the margin
SPEED_LOOKAHEAD = 0.1     # s ahead the speed profile is read
PLAN_DECEL = 2.5          # m/s^2 the profile slows down at (gentler than braking, so the car
                          # is already settled at the slow speed when the precise part comes)
SHARP_TURN = 0.7          # turns over this share of KAPPA_MAX are taken at TIGHT_SPEED
LAT_ERR_COEFF = 0.5       # how much being off the route can cut the speed (0 to 1)...
LAT_ERR_FULL = 0.3        # ...m off at which it cuts it all...
LAT_ERR_BEND = 0.8        # ...and more so in bends (per 1/m)


class ObstacleAvoidance(Node):

    def __init__(self, **kwargs):
        super().__init__('obstacle_avoidance', **kwargs)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        # How far ahead of the odometry's position the LiDAR sits (the course simulator's
        # scan_distance_to_base_link).
        self.declare_parameter('lidar_offset', 0.275)
        self.declare_parameter('centerline', os.path.join(
            get_package_share_directory('gap_follow'), 'config', 'levine_centerline.csv'))

        # The racing line planned offline from the map (make_racing_line), if there is one:
        # the route keeps to it unless something not on the map is in the way.
        self.declare_parameter('racing_line', '')

        self.track = np.loadtxt(self.get_parameter('centerline').value, delimiter=',',
                                comments='#')
        line_file = self.get_parameter('racing_line').value
        self.racing_line = None
        if line_file:
            line = np.loadtxt(line_file, delimiter=',', comments='#')
            self.racing_line = (line[:, 0], line[:, 1])
        self.set_direction(reverse=False)

        self.pose = None          # x, y, yaw, speed
        self.index = None         # nearest centerline point
        self.speed = 0.0
        self.steering = 0.0       # last steering command
        self.latency = LATENCY    # s, measured as the node runs
        self.prev_stamp = None
        self.direction_known = False
        self.last_plan = None     # (s, d) of the previous path

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.odom_callback, 1)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 1)

    # ---- the track -------------------------------------------------------------------

    def set_direction(self, reverse):
        """Use the centerline as given (counter-clockwise) or the other way round."""
        (self.cx, self.cy, self.tx, self.ty, self.left, self.right, self.s,
         self.length) = centerline_arrays(self.track, reverse)
        heading = np.unwrap(np.arctan2(self.ty, self.tx))
        turn = np.roll(heading, -1) - np.roll(heading, 1)
        turn = (turn + np.pi) % (2 * np.pi) - np.pi
        self.curvature = turn / (2.0 * (self.length / len(self.cx)))
        self.index = None
        # The racing line was planned counter-clockwise; the other way round, use the centre.
        self.line = None if reverse else self.racing_line

    def at(self, s):
        """Centerline position, tangent and widths at distances s along the track."""
        p = self.length
        ext_s = np.concatenate([self.s, [p]])

        def interp(v):
            return np.interp(np.mod(s, p), ext_s, np.concatenate([v, v[:1]]))
        tx, ty = interp(self.tx), interp(self.ty)
        norm = np.hypot(tx, ty)
        return (interp(self.cx), interp(self.cy), tx / norm, ty / norm,
                interp(self.left), interp(self.right))

    def nearest_index(self, x, y):
        if self.index is None:
            candidates = np.arange(len(self.cx))
        else:
            candidates = (self.index + np.arange(-30, 31)) % len(self.cx)
        i = candidates[np.argmin((self.cx[candidates] - x) ** 2 + (self.cy[candidates] - y) ** 2)]
        if (self.cx[i] - x) ** 2 + (self.cy[i] - y) ** 2 > 1.5 ** 2 and self.index is not None:
            self.index = None             # lost (e.g. the car was reset): search everywhere
            return self.nearest_index(x, y)
        return int(i)

    def to_frenet(self, px, py, first, count):
        """s (from the car's nearest centerline point, self.index) and d of points, matched
        to centerline points first .. first+count."""
        idx = (first + np.arange(count)) % len(self.cx)
        dx = px[:, None] - self.cx[idx][None, :]
        dy = py[:, None] - self.cy[idx][None, :]
        j = np.argmin(dx ** 2 + dy ** 2, axis=1)
        rows = np.arange(len(px))
        tx, ty = self.tx[idx][j], self.ty[idx][j]
        s0 = np.mod(self.s[idx][j] - self.s[self.index], self.length)
        s0 = np.where(s0 > self.length / 2, s0 - self.length, s0)
        s = s0 + dx[rows, j] * tx + dy[rows, j] * ty
        d = -dx[rows, j] * ty + dy[rows, j] * tx
        return s, d

    # ---- callbacks -------------------------------------------------------------------

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                     msg.twist.twist.linear.x)

    def scan_callback(self, msg):
        if self.pose is None:
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.prev_stamp is None else max(0.0, now - self.prev_stamp)
        self.prev_stamp = now
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        steering, speed, path = self.plan(ranges, angles, *self.pose, dt)
        self.publish(steering, speed, msg.header.stamp, path)
        # Measure the lag rather than assume it: from the scan to this command going out,
        # plus the simulator's wait for its next physics step. A slow PC or a busy moment
        # then just means the node plans further ahead.
        delay = (self.get_clock().now().nanoseconds * 1e-9) - now
        if 0.0 <= delay < 1.0:
            self.latency = float(np.clip(0.9 * self.latency + 0.1 * (delay + PHYSICS_STEP),
                                         LATENCY_MIN, LATENCY_MAX))

    # ---- planning --------------------------------------------------------------------

    def plan(self, ranges, angles, x, y, yaw, velocity, dt):
        """Steering angle, speed and the planned path (x, y) for one scan."""
        # A command takes effect LATENCY after the scan (planning, messaging, the simulator's
        # next step): plan from where the car will be by then. The scan stays where it was
        # taken, so it is moved into the frame of the LiDAR's predicted position.
        ranges = np.asarray(ranges, dtype=float)
        offset = self.get_parameter('lidar_offset').value
        v = max(velocity, 0.0)
        turn = v * np.tan(self.steering) / WHEELBASE * self.latency
        ahead = v * self.latency
        nx = x + ahead * np.cos(yaw + turn / 2.0)
        ny = y + ahead * np.sin(yaw + turn / 2.0)
        nyaw = yaw + turn
        old_lidar = np.array([x + offset * np.cos(yaw), y + offset * np.sin(yaw)])
        new_lidar = np.array([nx + offset * np.cos(nyaw), ny + offset * np.sin(nyaw)])
        wx = old_lidar[0] + ranges * np.cos(yaw + angles) - new_lidar[0]
        wy = old_lidar[1] + ranges * np.sin(yaw + angles) - new_lidar[1]
        ranges = np.hypot(wx, wy)
        angles = np.arctan2(wy, wx) - nyaw
        # The car is planned for as its footprint's centre, which in the course simulator is
        # the LiDAR (its collision box is centred there); the motion is still simulated from
        # the car's own reference point.
        self.reference = (nx, ny)
        return self.plan_from(ranges, angles, new_lidar[0], new_lidar[1], nyaw, velocity, dt)

    def plan_from(self, ranges, angles, x, y, yaw, velocity, dt):
        if not self.direction_known:
            i = self.nearest_index(x, y)
            if np.cos(yaw) * self.tx[i] + np.sin(yaw) * self.ty[i] < 0.0:
                self.set_direction(reverse=True)
            self.direction_known = True
        self.index = self.nearest_index(x, y)
        car_s, car_d = self.to_frenet(np.array([x]), np.array([y]),
                                      (self.index - 5) % len(self.cx), 11)
        car_s, car_d = car_s[0], car_d[0]

        horizon = min(HORIZON_MAX, HORIZON + self.speed ** 2 / (2.0 * BRAKE_DECEL))
        bins = int(np.ceil(horizon / CELL_S)) + 1
        bin_s = car_s + np.arange(bins) * CELL_S
        d_max = 2.0
        cells = int(round(2 * d_max / CELL_D)) + 1
        cell_d = -d_max + np.arange(cells) * CELL_D

        # Blocked cells: off the track, or within reach of the car's body of a LiDAR point.
        bx, by, btx, bty, left, right = self.at(self.s[self.index] + bin_s)
        # The track's measured widths only bound the search (the LiDAR sees the real walls):
        # it keeps paths from wandering out through a doorway.
        blocked = (cell_d[None, :] > left[:, None] + EDGE_SLACK) | \
                  (cell_d[None, :] < -right[:, None] - EDGE_SLACK)

        # Room around each grid cell, measured on the map itself (track coordinates bend
        # around corners and would misjudge distances there): a distance map of the LiDAR
        # points, looked up at each cell's position.
        cell_x = bx[:, None] - cell_d[None, :] * bty[:, None]
        cell_y = by[:, None] + cell_d[None, :] * btx[:, None]
        ok = np.isfinite(ranges) & (ranges > 0.05) & (ranges < horizon + 2.0)
        px = x + ranges[ok] * np.cos(yaw + angles[ok])
        py = y + ranges[ok] * np.sin(yaw + angles[ok])
        # The LiDAR only sees an obstacle's near face: the SHADOW behind each hit is unknown.
        # Routes avoid it when they can (or they'd plan through the back of a box), but may
        # use it when it's the only way on, slowly enough to stop before it.
        depth = np.arange(RASTER, SHADOW + 1e-9, RASTER)
        reach = ranges[ok][:, None] + depth[None, :]
        sx = (x + reach * np.cos(yaw + angles[ok])[:, None]).ravel()
        sy = (y + reach * np.sin(yaw + angles[ok])[:, None]).ravel()
        # The car as three discs along its length, pointing the way the car will be going
        # there: along the previous plan further out, turning from the car's own heading over
        # the first HEADING_BLEND of the route (it can't swing round at once).
        car_dir = np.arctan2(bty, btx) + self.planned_heading(bin_s)
        offset = np.arctan2(np.sin(car_dir - yaw), np.cos(car_dir - yaw))
        car_dir = yaw + offset * np.clip((bin_s - car_s) / HEADING_BLEND, 0.0, 1.0)
        dir_x, dir_y = np.cos(car_dir)[:, None], np.sin(car_dir)[:, None]
        wanted = ~blocked
        self.distances = DistanceMap(px, py, cell_x[wanted], cell_y[wanted])
        shadows = DistanceMap(sx, sy, cell_x[wanted], cell_y[wanted])
        clearance = np.zeros(cell_x.shape)
        unseen = np.zeros(cell_x.shape)
        for dmap, out in ((self.distances, clearance), (shadows, unseen)):
            out[wanted] = np.min([dmap.at((cell_x + o * dir_x)[wanted],
                                          (cell_y + o * dir_y)[wanted]) for o in DISCS], axis=0)
        # Only touching (within HARD_MARGIN) is ruled out. Cutting into MARGIN costs a lot
        # and slows the car right down, but never leaves it with no way to go. Right by the
        # car the limit eases by up to START_SLACK (the discs are a little fatter than the
        # car), so a car that has drifted close to something can always ease away from it.
        ease = START_SLACK * (1.0 - np.clip((bin_s - car_s) / START_EASE, 0.0, 1.0))
        blocked |= clearance < DISC_RADIUS + HARD_MARGIN - ease[:, None]
        blocked[0] = False
        spare = clearance - DISC_RADIUS - MARGIN
        near = np.clip(NEAR - spare, 0.0, NEAR) + MARGIN_WEIGHT * np.clip(-spare, 0.0, MARGIN)
        in_shadow = unseen < DISC_RADIUS + MARGIN

        # Keep to the racing line (or the centerline, without one).
        target = np.zeros(bins)
        if self.line is not None:
            line_s, line_d = self.line
            target = np.interp(np.mod(self.s[self.index] + bin_s, self.length),
                               np.append(line_s, self.length), np.append(line_d, line_d[0]))
        weight = LINE_WEIGHT if self.line is not None else CENTER_WEIGHT
        cost = weight * (cell_d[None, :] - target[:, None]) ** 2 + NEAR_WEIGHT * near \
            + SHADOW_COST * in_shadow
        if self.last_plan is not None:
            # Stick with the last plan unless there's a reason not to: flipping between two
            # equally good sides of an obstacle would make the car weave.
            last_s, last_d = self.last_plan
            ahead = np.mod(last_s - self.s[self.index], self.length)
            ahead = np.where(ahead > self.length / 2, ahead - self.length, ahead)
            covered = (bin_s >= ahead.min()) & (bin_s <= ahead.max())
            order = np.argsort(ahead)
            last_at = np.interp(bin_s, ahead[order], last_d[order])
            cost += np.where(covered[:, None],
                             STICK_WEIGHT * (cell_d[None, :] - last_at[:, None]) ** 2, 0.0)
        cost = np.where(blocked, np.inf, cost)

        # With a racing line that the LiDAR shows clear, follow the line itself, as ForzaETH's
        # controller does: it's a fixed reference, so the car is steered back onto it when it
        # drifts. A route replanned from wherever the car is would take each drift as its new
        # start, and drifts add up (a few cm too wide into a tight corner leaves no way
        # round). Only when something not on the map is in the way does the car route round
        # it, and it rejoins the line as soon as it's clear again.
        on_line = None
        if self.line is not None:
            on_line = np.clip(np.round((target + d_max) / CELL_D).astype(int), 0, cells - 1)
            if blocked[np.arange(1, bins), on_line[1:]].any():
                on_line = None
        if on_line is not None:
            path_d, reached = target.copy(), bins - 1
        else:
            path_d, reached = self.route(cost, cell_d, bx, by, btx, bty, car_d, x, y, yaw)
        self.last_plan = (self.s[self.index] + bin_s[:reached + 1], path_d.copy())
        path_x = bx[:reached + 1] - path_d * bty[:reached + 1]
        path_y = by[:reached + 1] + path_d * btx[:reached + 1]

        steering = self.track_path(path_x, path_y, x, y, yaw)
        # Smooth the steering (the route is replanned every scan and moves a little each
        # time), before the check below so it checks what is actually sent.
        if dt > 0.0:
            blend = dt / (STEER_SMOOTHING + dt)
            steering = self.steering + blend * (steering - self.steering)
        room = self.room_on_path(clearance, left, right, path_d, cell_d, d_max, reached)
        on_route = np.clip(np.round((path_d + d_max) / CELL_D).astype(int), 0, cells - 1)
        unseen_rows = np.flatnonzero(in_shadow[np.arange(reached + 1), on_route][1:])
        # The racing line was planned on the map, so space the LiDAR can't see yet along it
        # isn't a reason to slow down (an obstacle not on the map shows up in time to stop:
        # check_command brakes for it).
        unseen_from = unseen_rows[0] + 1 if len(unseen_rows) and on_line is None else None
        speed = self.choose_speed(path_x, path_y, room, unseen_from, velocity)
        for factor in (1.0, 0.7, 0.45, 0.25):
            # The slowest tries are kept: a slower car always has more room to stop.
            checked = self.check_command(steering, speed * factor, velocity, x, y, yaw,
                                         path_x, path_y)
            if checked is not None:
                steering, speed = checked
                break
        else:
            steering, speed = self.roomiest_command(speed)
        # Never swing the steering faster than STEER_SLEW: one odd scan can't jerk it across.
        if dt > 0.0:
            steering = float(np.clip(steering, self.steering - STEER_SLEW * dt,
                                     self.steering + STEER_SLEW * dt))
        self.speed = speed
        self.steering = steering
        return steering, speed, (path_x, path_y)

    def check_command(self, wanted, speed, velocity, x, y, yaw, path_x, path_y):
        """Steering and speed that keep the car clear of everything for the next moment.

        Simulates the car (bicycle model, with the steering turning at its real rate) under
        the wanted steering and under alternatives: hold one angle for HOLD seconds, then
        another. Sends the wanted one if the car's footprint stays clear, otherwise the
        clear alternative closest to it that also ends near the path; None if none is
        clear at this speed (the roomiest is kept for roomiest_command).
        """
        max_steer = MAX_STEERING
        firsts = np.append(np.linspace(-max_steer, max_steer, 15), wanted)
        seconds = np.linspace(-max_steer, max_steer, 5)
        first = np.repeat(firsts, len(seconds))
        second = np.tile(seconds, len(firsts))
        # Drive on at this speed for HOLD, then brake to a stop: if that stays clear, the car
        # can always stop in time if the next scans show something new.
        v0 = max(0.5, min(speed, max(velocity, 0.0) + SPEED_GAIN_LIMIT))
        horizon = max(CHECK_TIME, HOLD + v0 / BRAKE_DECEL)
        steps = int(np.ceil(horizon / CHECK_DT))
        t = (np.arange(steps) + 1) * CHECK_DT
        braking = np.maximum(np.where(t <= HOLD, v0, v0 - BRAKE_DECEL * (t - HOLD)), 0.3)
        room, px, py = self.simulate(first, second, braking, x, y, yaw)
        # ...and the car mustn't end up nose-first in a corner it can't drive out of: the
        # same steering, driving on another KEEP_GOING instead of stopping, has to stay
        # clear too (the simulator's car can't reverse out).
        going = np.full(int(np.ceil((HOLD * v0 + KEEP_GOING) / (v0 * CHECK_DT))), v0)
        room_going, _, _ = self.simulate(first, second, going, x, y, yaw)

        i = int(np.argmax(np.minimum(room, room_going)))
        self.roomiest = (float(first[i]), float(room[i]))
        safe = room >= CHECK_MARGIN
        if (safe & (room_going >= CHECK_MARGIN)).any():
            safe &= room_going >= CHECK_MARGIN
        wanted_rows = np.flatnonzero(np.isclose(first, wanted))
        if safe[wanted_rows].any():
            return wanted, speed
        clear = safe
        if not clear.any():
            return None
        off_path = np.min((px[:, -1, None] - path_x[None, :]) ** 2 +
                          (py[:, -1, None] - path_y[None, :]) ** 2, axis=1)
        # Closest to what's wanted, ending near the route, and close to the last command
        # (so the choice doesn't flip from one side to the other between scans).
        cost = (first - wanted) ** 2 + CHECK_PATH_WEIGHT * off_path \
            + CHECK_STICK_WEIGHT * (first - self.steering) ** 2
        i = np.flatnonzero(clear)[np.argmin(cost[clear])]
        return float(first[i]), speed

    def simulate(self, first, second, speeds, x, y, yaw):
        """Drive the car from its pose at speeds[k] on step k (CHECK_DT long), steering
        toward first[i] for HOLD seconds and then second[i], the steering turning at
        STEER_RATE. Returns each run's least room around the footprint, and its x and y."""
        steps = len(speeds)
        t = (np.arange(steps) + 1) * CHECK_DT
        target = np.where(t[None, :] <= HOLD, first[:, None], second[:, None])
        delta = np.empty_like(target)
        current = np.full(len(first), self.steering)
        for k in range(steps):
            current = current + np.clip(target[:, k] - current,
                                        -STEER_RATE * CHECK_DT, STEER_RATE * CHECK_DT)
            delta[:, k] = current
        v = speeds[None, :]
        # The car's reference point moves by the bicycle model; the footprint (and the
        # returned position) is lidar_offset ahead of it.
        offset = self.get_parameter('lidar_offset').value
        rx, ry = self.reference
        heading = yaw + np.cumsum(v * np.tan(delta) / WHEELBASE * CHECK_DT, axis=1)
        px = rx + np.cumsum(v * np.cos(heading) * CHECK_DT, axis=1) + offset * np.cos(heading)
        py = ry + np.cumsum(v * np.sin(heading) * CHECK_DT, axis=1) + offset * np.sin(heading)
        # The first COMMITTED seconds can't be changed any more (the lag), so they don't count.
        skip = int(round(COMMITTED / CHECK_DT))
        room = np.min([self.distances.at(px + o * np.cos(heading), py + o * np.sin(heading))
                       for o in DISCS], axis=0)[:, skip:].min(axis=1) - DISC_RADIUS
        return room, px, py

    def roomiest_command(self, speed):
        """Nothing checked out: steer for the most room, and creep (or stop if touching)."""
        steering, room = self.roomiest
        return steering, (min(speed, CREEP_SPEED) if room > -START_SLACK else 0.0)

    def route(self, cost, cell_d, bx, by, btx, bty, car_d, x, y, yaw):
        """The cheapest route through the grid that the car can actually drive: offsets d
        for rows 0..reached (past `reached` there is no way on, so the car must stop there).

        Searches over the offset and its slope (the offset one step back) together,
        ROUTE_STEP rows at a time. That fixes the last three points of every partial route,
        so the route's turning radius is measured exactly, on the map (track coordinates
        bend at corners), and kept within KAPPA_MAX: the car's tightest turn. The route
        starts at the car, the way it is pointing. (The turning-radius and start-heading
        limits follow the UNICORN racing stack's SQP avoidance planner.)
        """
        # Steps of ROUTE_STEP grid rows, longer on long (fast) routes so the search stays quick.
        stride = max(ROUTE_STEP, int(np.ceil(cost.shape[0] / ROUTE_STEPS_MAX)))
        ds = stride * CELL_S
        rows = np.arange(0, cost.shape[0], stride)
        group = int(ROUTE_CELLS)
        pairs = (len(cell_d) - 1) // group
        offsets = cell_d[:group * pairs].reshape(pairs, group).mean(axis=1)
        # A row costs the worst of the grid rows and cells it stands for.
        half = stride // 2
        row_cost = np.array([cost[max(0, b - half):b + half + 1, :group * pairs].max(axis=0)
                             for b in rows])
        row_cost = row_cost.reshape(len(rows), pairs, group).max(axis=2)
        # Only the span of cells that is ever open needs searching.
        open_cells = np.flatnonzero(np.isfinite(row_cost[1:]).any(axis=0))
        if len(open_cells) == 0:
            return np.full(1, car_d), 0
        start = int(np.argmin(np.abs(offsets - car_d)))
        lo, hi = min(open_cells[0], start), max(open_cells[-1], start) + 1
        offsets, row_cost, start = offsets[lo:hi], row_cost[:, lo:hi], start - lo
        pairs = hi - lo
        # Every route point on the map; row 0 is the car itself.
        px = bx[rows][:, None] - offsets[None, :] * bty[rows][:, None]
        py = by[rows][:, None] + offsets[None, :] * btx[rows][:, None]
        px[0], py[0] = x, y

        max_slope = int(SLOPE * ds / (group * CELL_D))
        slopes = np.arange(-max_slope, max_slope + 1)          # cells moved per row
        nslopes = len(slopes)
        cells = np.arange(pairs)[:, None]
        prev_cell = cells - slopes[None, :]                     # one row back, per slope
        prev_ok = (prev_cell >= 0) & (prev_cell < pairs)
        prev_cell = np.clip(prev_cell, 0, pairs - 1)

        def turn(ax, ay, bxx, byy, cx, cy):
            """Signed curvature through three points (1/m)."""
            cross = (bxx - ax) * (cy - byy) - (byy - ay) * (cx - bxx)
            sides = np.hypot(bxx - ax, byy - ay) * np.hypot(cx - bxx, cy - byy) * \
                np.hypot(cx - ax, cy - ay)
            return 2.0 * cross / np.maximum(sides, 1e-9)

        # Row 1: from a point just behind the car, through the car (its heading), to each cell.
        k1 = turn(x - ds * np.cos(yaw), y - ds * np.sin(yaw), x, y, px[1], py[1])
        slope1 = np.arange(pairs) - start
        total = np.full((pairs, nslopes), np.inf)
        ok = (np.abs(k1) <= KAPPA_MAX) & (np.abs(slope1) <= max_slope)
        total[np.flatnonzero(ok), slope1[ok] + max_slope] = \
            row_cost[1][ok] + BEND_COST * k1[ok] ** 2 * ds
        came_from = [np.zeros((pairs, nslopes), dtype=np.int8)]
        reached = 1 if np.isfinite(total).any() else 0
        for r in range(2, len(rows) if reached else 0):
            best = np.full_like(total, np.inf)
            choice = np.zeros(total.shape, dtype=np.int8)
            for change in range(-MAX_SLOPE_CHANGE, MAX_SLOPE_CHANGE + 1):
                prev_slope = slopes - change                   # slope one row back
                slope_ok = np.abs(prev_slope) <= max_slope
                back2 = prev_cell - prev_slope[None, :]          # two rows back
                ok = prev_ok & slope_ok[None, :] & (back2 >= 0) & (back2 < pairs)
                back2 = np.clip(back2, 0, pairs - 1)
                k = turn(px[r - 2][back2], py[r - 2][back2], px[r - 1][prev_cell],
                         py[r - 1][prev_cell], px[r][cells], py[r][cells])
                ok &= np.abs(k) <= KAPPA_MAX
                cand = total[prev_cell, np.clip(prev_slope + max_slope, 0, nslopes - 1)[None, :]]
                cand = np.where(ok, cand + BEND_COST * k ** 2 * ds, np.inf)
                better = cand < best
                best[better] = cand[better]
                choice[better] = change
            best += row_cost[r][:, None]
            if not np.isfinite(best).any():
                break
            total = best
            came_from.append(choice)
            reached = r

        route_d = np.empty(reached + 1)
        route_d[0] = car_d
        if reached:
            c, j = np.unravel_index(int(np.argmin(total)), total.shape)
            for r in range(reached, 0, -1):
                route_d[r] = offsets[c]
                change = came_from[r - 1][c, j]
                c, j = c - slopes[j], j - change
        last_bin = rows[reached]
        if reached < 2:
            return np.interp(np.arange(last_bin + 1), rows[:reached + 1], route_d), last_bin
        # A smooth curve through the route's points (straight lines between them would
        # leave kinks the car can't follow).
        curve = CubicSpline(rows[:reached + 1], route_d, bc_type='natural')
        return curve(np.arange(last_bin + 1)), last_bin

    def planned_heading(self, bin_s):
        """Angle between the previous plan and the track at each row (0 where there is
        none): atan of the offset's rate of change, stretched by the bend."""
        if self.last_plan is None or len(self.last_plan[0]) < 3:
            return np.zeros(len(bin_s))
        last_s, last_d = self.last_plan
        ahead = np.mod(last_s - self.s[self.index], self.length)
        ahead = np.where(ahead > self.length / 2, ahead - self.length, ahead)
        order = np.argsort(ahead)
        ahead, last_d = ahead[order], last_d[order]
        slope = np.gradient(last_d, ahead)
        bend = np.interp(np.mod(self.s[self.index] + ahead, self.length), self.s,
                         self.curvature)
        angle = np.arctan2(slope, np.maximum(0.2, 1.0 - bend * last_d))
        covered = (bin_s >= ahead[0]) & (bin_s <= ahead[-1])
        return np.where(covered, np.interp(bin_s, ahead, angle), 0.0)

    def room_on_path(self, clearance, left, right, path_d, cell_d, d_max, reached):
        """Room to spare around the car (beyond its footprint and margin) at each row of the
        path (m)."""
        rows = np.arange(reached + 1)
        c = np.clip(np.round((path_d + d_max) / CELL_D).astype(int), 0, len(cell_d) - 1)
        room = clearance[rows, c] - DISC_RADIUS - MARGIN
        room[0] = room[1] if reached >= 1 else 0.0
        return room

    def track_path(self, path_x, path_y, x, y, yaw):
        """Steering to follow the path closely, judged where the path is drawn for (the
        LiDAR point, x, y): the path's own bend, plus turning to match its direction, plus
        closing any sideways gap to it."""
        self.route_gap = self.route_bend = self.route_heading_error = 0.0
        if len(path_x) < 3:
            return 0.0
        i = int(np.argmin((path_x - x) ** 2 + (path_y - y) ** 2))
        i = int(np.clip(i, 1, len(path_x) - 2))
        tx, ty = path_x[i + 1] - path_x[i - 1], path_y[i + 1] - path_y[i - 1]
        path_heading = np.arctan2(ty, tx)
        # Bend of the path where the car will be once this command has taken effect (the
        # measured lag, plus the steering filter) and a little further (feed-forward), over
        # a chord of BEND_CHORD: the route is straight pieces between its steps, so points
        # closer together would read either no bend or a spike.
        preview = self.latency + STEER_SMOOTHING + CONTROL_PREVIEW
        half = max(1, int(round(BEND_CHORD / 2.0 / CELL_S)))
        j = int(np.clip(i + int(round(self.speed * preview / CELL_S)), 1, len(path_x) - 2))
        ax, ay = path_x[max(0, j - half)], path_y[max(0, j - half)]
        bx, by = path_x[j], path_y[j]
        cx, cy = path_x[min(len(path_x) - 1, j + half)], path_y[min(len(path_x) - 1, j + half)]
        cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        sides = np.hypot(bx - ax, by - ay) * np.hypot(cx - bx, cy - by) * np.hypot(cx - ax, cy - ay)
        bend = 2.0 * cross / max(sides, 1e-9)
        # Sideways gap: positive when the LiDAR point is left of the path.
        gap = (-(x - path_x[i]) * ty + (y - path_y[i]) * tx) / max(np.hypot(tx, ty), 1e-9)
        # The LiDAR point is lidar_offset ahead of the point the car turns about, so on a
        # path of radius r the car turns on sqrt(r^2 - offset^2). And while the car turns,
        # the point doesn't move straight ahead but slips sideways into the turn: compare
        # the path's direction with the way the point is moving now, under the steering
        # already sent (not the turn it's heading into, or the car would turn in late).
        offset = self.get_parameter('lidar_offset').value
        radius = 1.0 / max(abs(bend), 1e-3)
        turning = np.sqrt(max(radius ** 2 - offset ** 2, (WHEELBASE / np.tan(MAX_STEERING)) ** 2))
        feed_forward = np.copysign(np.arctan(WHEELBASE / turning), bend)
        moving = yaw + np.arctan(offset * np.tan(self.steering) / WHEELBASE)
        heading_error = np.arctan2(np.sin(path_heading - moving), np.cos(path_heading - moving))
        steering = feed_forward + heading_error - np.arctan2(GAP_GAIN * gap, self.speed + 1.0)
        # Kept for choose_speed: how far off the route the car is, and how it's pointing.
        self.route_gap, self.route_bend, self.route_heading_error = gap, bend, heading_error
        return float(np.clip(steering, -MAX_STEERING, MAX_STEERING))

    def choose_speed(self, path_x, path_y, room, unseen_from, velocity):
        """Speed for the route, in the style of ForzaETH's MAP controller (which the UNICORN
        stack's controller builds on): a speed profile along the route, read a little ahead
        of the car, then scaled down while the car is off its route.

        Each point of the route gets the most speed it can be taken at: what its bend holds,
        and what the precision there allows (TIGHT_SPEED plus SPEED_PER_ROOM per m of room
        around the car, so tight slots are slow and open hallway is fast). Unseen space is
        taken at UNSEEN_SPEED and the car stops by the route's end. A backward pass makes
        every point reachable by braking, a forward pass by accelerating from the car's
        speed now: slow exactly where precision is needed, fast everywhere else.
        """
        max_speed = self.get_parameter('max_speed').value
        n = len(path_x)
        cap = np.full(n, max_speed)
        if n >= 7:
            ax, ay = path_x[:-6], path_y[:-6]
            bx, by = path_x[3:-3], path_y[3:-3]
            cx, cy = path_x[6:], path_y[6:]
            cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
            sides = np.hypot(bx - ax, by - ay) * np.hypot(cx - bx, cy - by) * \
                np.hypot(cx - ax, cy - ay)
            curvature = 2.0 * np.abs(cross) / np.maximum(sides, 1e-9)
            cap[3:-3] = np.minimum(cap[3:-3], np.sqrt(LATERAL_ACCEL / np.maximum(curvature, 1e-3)))
            # Turns near the car's tightest need precision too.
            sharp = curvature > SHARP_TURN * KAPPA_MAX
            cap[3:-3] = np.where(sharp, np.minimum(cap[3:-3], TIGHT_SPEED), cap[3:-3])
        cap = np.minimum(cap, np.maximum(CREEP_SPEED, TIGHT_SPEED + SPEED_PER_ROOM * room))
        if unseen_from is not None:
            cap[unseen_from:] = np.minimum(cap[unseen_from:], UNSEEN_SPEED)
        cap[-1] = 0.0                                     # nothing known past the route
        step = np.hypot(np.diff(path_x), np.diff(path_y)) if n > 1 else np.zeros(0)

        profile = cap.copy()
        for i in range(n - 2, -1, -1):                    # be slow in time for what's ahead
            profile[i] = min(profile[i], np.sqrt(profile[i + 1] ** 2 + 2.0 * PLAN_DECEL * step[i]))
        profile[0] = min(profile[0], max(velocity, 0.0) + ACCEL * CHECK_DT)
        for i in range(1, n):                             # can't gain speed faster than ACCEL
            profile[i] = min(profile[i], np.sqrt(profile[i - 1] ** 2 + 2.0 * ACCEL * step[i - 1]))

        # Read the profile where the car will be a moment from now (the command acts late).
        along = np.concatenate([[0.0], np.cumsum(step)])
        ahead = max(velocity, 0.0) * SPEED_LOOKAHEAD
        speed = float(np.interp(ahead, along, profile)) if n > 1 else 0.0

        # Off the route, slow down until it's caught again, more so in a bend
        # (ForzaETH's speed_adjust_lat_err), and when pointing well away from it
        # (speed_adjust_heading).
        off = np.clip(abs(self.route_gap) / LAT_ERR_FULL, 0.0, 1.0)
        bend = np.clip(abs(self.route_bend) * LAT_ERR_BEND, 0.0, 1.0)
        speed *= 1.0 - LAT_ERR_COEFF + LAT_ERR_COEFF * np.exp(-2.0 * off * (0.5 + bend))
        wrong = abs(self.route_heading_error)
        if wrong > np.radians(20.0):
            speed *= max(0.5, 1.0 - 0.5 * wrong / (np.pi / 2))
        return max(0.0, speed)

    def publish(self, steering, speed, stamp, path=None):
        msg = AckermannDriveStamped()
        msg.header.stamp = stamp
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_publisher.publish(msg)
        if path is not None and self.path_publisher.get_subscription_count() > 0:
            out = Path()
            out.header.frame_id = 'map'
            out.header.stamp = stamp
            for px, py in zip(*path):
                pose = PoseStamped()
                pose.header = out.header
                pose.pose.position.x, pose.pose.position.y = float(px), float(py)
                pose.pose.orientation.w = 1.0
                out.poses.append(pose)
            self.path_publisher.publish(out)


def centerline_arrays(track, reverse=False):
    """From a centerline file's rows (x, y, width right, width left): x, y, unit tangent x and
    y, width left and right, distance along it at each point, and its length."""
    x, y = track[:, 0], track[:, 1]
    right, left = track[:, 2], track[:, 3]
    if reverse:
        x, y, right, left = x[::-1], y[::-1], left[::-1], right[::-1]
    step = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    tx, ty = np.roll(x, -1) - np.roll(x, 1), np.roll(y, -1) - np.roll(y, 1)
    norm = np.hypot(tx, ty)
    s = np.concatenate([[0.0], np.cumsum(step)[:-1]])
    return x, y, tx / norm, ty / norm, left, right, s, step.sum()


def load_centerline(path, reverse=False):
    return centerline_arrays(np.loadtxt(path, delimiter=',', comments='#'), reverse)


class DistanceMap:
    """Distance to the nearest of the points (px, py), in m, anywhere around the area the
    positions (ax, ay) cover: a map with RASTER-sized squares, so accurate to about half a
    square."""

    def __init__(self, px, py, ax, ay):
        if len(ax) == 0:
            ax, ay = np.zeros(1), np.zeros(1)
        self.x0, x1 = ax.min() - 1.0, ax.max() + 1.0
        self.y0, y1 = ay.min() - 1.0, ay.max() + 1.0
        inside = (px > self.x0) & (px < x1) & (py > self.y0) & (py < y1)
        self.cols = int(np.ceil((x1 - self.x0) / RASTER)) + 1
        self.rows = int(np.ceil((y1 - self.y0) / RASTER)) + 1
        free = np.ones((self.rows, self.cols), dtype=bool)
        free[((py[inside] - self.y0) / RASTER).astype(int),
             ((px[inside] - self.x0) / RASTER).astype(int)] = False
        self.dist = np.full(free.shape, np.inf) if free.all() \
            else distance_transform_edt(free) * RASTER

    def at(self, qx, qy):
        r = np.clip(((qy - self.y0) / RASTER).astype(int), 0, self.rows - 1)
        c = np.clip(((qx - self.x0) / RASTER).astype(int), 0, self.cols - 1)
        return self.dist[r, c]


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Handle Ctrl+C here so the node can publish a stop command on the way out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _interrupt)
    node = ObstacleAvoidance()
    try:
        # Wake every 0.1 s even when no scans come (the simulator may already have
        # stopped), so Ctrl+C is always noticed and the node never hangs on.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.drive_publisher.publish(AckermannDriveStamped())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
