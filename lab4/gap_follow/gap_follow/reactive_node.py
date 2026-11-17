import signal

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# How fast the car can turn and brake (the RoboRacer car in the simulator). The speed is
# always kept within these, so max_speed can be set to anything.
WHEELBASE = 0.33          # m
LATERAL_ACCEL = 6.0       # m/s^2 of grip in a turn
BRAKE_DECEL = 5.0         # m/s^2 of braking
CORNER_SPEED = 2.0        # m/s to be down to...
CORNER_MARGIN = 1.0       # ...this many metres before something ahead
SPEED_UP = 5.0            # m/s^2: how fast the car may gain speed

# Above 2 m/s, the side-clearance check and the margin kept from obstacle edges grow by this
# much per m/s: a fast car follows its aim point less closely and would cut corners.
SIDE_GROWTH = 0.08        # m per m/s
MARGIN_GROWTH = 0.12      # m per m/s

# Faster than SMOOTH_ABOVE, steering is smoothed so the car doesn't twitch.
SMOOTH_ABOVE = 2.5        # m/s
STEERING_BLEND = 0.35     # share of each new steering command when smoothing


class ReactiveFollowGap(Node):
    """Follow the gap: drive toward the gap the car fits through that leads furthest forward."""

    def __init__(self):
        super().__init__('reactive_node')

        # Top speed, on straights. Set it to anything: the car slows for turns and anything
        # ahead by itself (75% of it in gentle turns, 50% in sharp ones or when close).
        self.declare_parameter('max_speed', 2.0)

        self.declare_parameter('fov_deg', 90.0)            # look this far left and right
        self.declare_parameter('max_range', 8.0)           # clip far readings
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('car_half_width', 0.25)     # half the car's width plus a margin
        self.declare_parameter('disparity_threshold', 0.4)
        self.declare_parameter('bubble_radius', 0.3)
        # Walls closer than this are not free space, so a nearby wall at a corner can't
        # out-vote the hallway the car should turn into.
        self.declare_parameter('gap_min_range', 2.0)
        self.declare_parameter('best_point_fraction', 0.9) # aim at the middle of the deepest part
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('max_steering', 0.4189)
        # Don't turn toward a side with a wall this close beside the car (stops corner cutting).
        self.declare_parameter('side_clearance', 0.3)
        self.declare_parameter('rear_clearance', 0.17)
        self.declare_parameter('slow_down_distance', 1.5) # slow down if anything is this close ahead

        self.speed = 0.0
        self.steering = 0.0
        self.prev_stamp = None

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def param(self, name):
        return self.get_parameter(name).value

    def preprocess_lidar(self, scan_msg):
        """Crop to the front field of view, clean up bad beams, smooth, then extend disparities.

        Returns the processed ranges and the angle of each one.
        """
        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

        fov = np.radians(self.param('fov_deg'))
        keep = np.abs(angles) <= fov
        ranges = ranges[keep]
        angles = angles[keep]

        max_range = self.param('max_range')
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, max_range)

        window = int(self.param('smoothing_window'))
        if window > 1:
            ranges = np.convolve(ranges, np.ones(window) / window, mode='same')

        ranges = self.extend_disparities(ranges, scan_msg.angle_increment)
        return ranges, angles

    def extend_disparities(self, ranges, angle_increment):
        """Where a near obstacle edge sits next to far space, widen the obstacle by the car's
        half-width so a gap only looks open if the car actually fits through it."""
        half_width = self.param('car_half_width') + MARGIN_GROWTH * max(0.0, self.speed - 2.0)
        threshold = self.param('disparity_threshold')
        extended = ranges.copy()

        for i in range(len(ranges) - 1):
            near = min(ranges[i], ranges[i + 1])
            if abs(ranges[i + 1] - ranges[i]) < threshold or near <= 0.0:
                continue
            beams = int(np.ceil((half_width / near) / angle_increment))
            if ranges[i] < ranges[i + 1]:
                end = min(len(ranges), i + 1 + beams)
                extended[i + 1:end] = np.minimum(extended[i + 1:end], near)
            else:
                start = max(0, i - beams + 1)
                extended[start:i + 1] = np.minimum(extended[start:i + 1], near)

        return extended

    def apply_bubble(self, ranges, angle_increment):
        """Zero every beam within bubble_radius of the closest point."""
        closest = int(np.argmin(ranges))
        distance = max(ranges[closest], 0.1)
        half_angle = np.arctan2(self.param('bubble_radius'), distance)
        beams = int(np.ceil(half_angle / angle_increment))

        bubbled = ranges.copy()
        bubbled[max(0, closest - beams):closest + beams + 1] = 0.0
        return bubbled

    def find_max_gap(self, ranges, angles):
        """Start and end index (inclusive) of the best run of free beams.

        Runs are compared by how far forward they lead (the largest range * cos(angle) in
        the run), not just by how many beams they span. A wide but shallow dead end, like
        the L-shaped pocket on levine_obs, then loses to a narrower passage that keeps going.
        """
        free = ranges > self.param('gap_min_range')
        progress = ranges * np.cos(angles)

        best_start, best_end, best_score = 0, -1, -np.inf
        start = None
        for i, is_free in enumerate(np.append(free, False)):
            if is_free and start is None:
                start = i
            elif not is_free and start is not None:
                score = progress[start:i].max()
                if score > best_score:
                    best_start, best_end, best_score = start, i - 1, score
                start = None

        return best_start, best_end

    def find_best_point(self, start, end, ranges, angles):
        """Middle of the part of the gap that gives the most forward progress.

        Scoring each beam by range * cos(angle) instead of range alone stops the car from
        U-turning at a corner into the hallway it just left, which can look deeper than the
        way ahead. When the best beams form several separate runs, take the run closest to
        straight ahead.
        """
        progress = ranges[start:end + 1] * np.cos(angles[start:end + 1])
        deep = start + np.flatnonzero(
            progress >= self.param('best_point_fraction') * progress.max())

        runs = np.split(deep, np.flatnonzero(np.diff(deep) > 1) + 1)
        run = min(runs, key=lambda r: np.abs(angles[r]).min())
        return int((run[0] + run[-1]) // 2)

    def lidar_callback(self, scan_msg):
        ranges, angles = self.preprocess_lidar(scan_msg)
        bubbled = self.apply_bubble(ranges, scan_msg.angle_increment)

        start, end = self.find_max_gap(bubbled, angles)
        if end < start:
            # Nothing clears gap_min_range (tight clutter): creep toward the deepest beam.
            best = int(np.argmax(bubbled))
        else:
            best = self.find_best_point(start, end, bubbled, angles)

        max_steering = self.param('max_steering')
        steering = float(np.clip(self.param('steering_gain') * angles[best],
                                 -max_steering, max_steering))

        left_blocked, right_blocked = self.side_blocked(scan_msg)
        if (steering > 0.0 and left_blocked) or (steering < 0.0 and right_blocked):
            steering = 0.0

        if self.speed > SMOOTH_ABOVE:
            steering = STEERING_BLEND * steering + (1.0 - STEERING_BLEND) * self.steering
        self.steering = steering

        now = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.prev_stamp is None else now - self.prev_stamp
        self.prev_stamp = now

        ahead = ranges[np.abs(angles) < np.radians(15.0)]
        ahead = ahead.min() if ahead.size else np.inf
        self.speed = self.choose_speed(steering, ahead, end < start, dt)
        self.publish(steering, self.speed, scan_msg)

    def choose_speed(self, steering, ahead, no_gap, dt):
        """Speed scaled to max_speed by how sharp the turn is, kept within what the car can
        turn and brake for."""
        max_speed = self.param('max_speed')
        steering_deg = abs(np.degrees(steering))
        if steering_deg < 10.0:
            speed = max_speed
        elif steering_deg < 20.0:
            speed = max_speed * 0.75
        else:
            speed = max_speed * 0.5
        if no_gap or ahead < self.param('slow_down_distance'):
            speed = min(speed, max_speed * 0.5)

        # A turn of radius r = wheelbase / tan(steering) holds at most sqrt(grip * r).
        if abs(steering) > 1e-3:
            speed = min(speed, np.sqrt(LATERAL_ACCEL * WHEELBASE / np.tan(abs(steering))))

        # Brake early enough to be down to CORNER_SPEED by CORNER_MARGIN before anything ahead.
        room = max(0.0, ahead - CORNER_MARGIN)
        speed = min(speed, np.sqrt(CORNER_SPEED ** 2 + 2.0 * BRAKE_DECEL * room))

        # Gain speed gradually at high speed; slowing down is never delayed.
        if speed > max(self.speed, SMOOTH_ABOVE) and dt > 0.0:
            speed = min(speed, max(self.speed, SMOOTH_ABOVE) + SPEED_UP * dt)
        return float(speed)

    def side_blocked(self, scan_msg):
        """Whether something sits right beside the car's body on the left / right.

        Looks alongside the car's body (the LiDAR sits near the front bumper): out to
        side_clearance beside the front half, where a corner the car is about to clip shows up
        first, and out to rear_clearance beside the rear half. The rear zone is narrower so an
        obstacle the car has just passed doesn't block the next turn. Nothing ahead of the
        bumper counts, or the check would block steering around an obstacle in front.
        Both zones widen with speed: a fast car swings wider and turns in too early otherwise.
        """
        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        ok = np.isfinite(ranges) & (ranges > 0.0)
        x = ranges[ok] * np.cos(angles[ok])
        y = np.abs(ranges[ok] * np.sin(angles[ok]))
        on_left = angles[ok] > 0.0

        extra = SIDE_GROWTH * max(0.0, self.speed - 2.0)
        front = (x >= -0.15) & (x < 0.15) & (y < self.param('side_clearance') + extra)
        rear = (x > -0.4) & (x < -0.15) & (y < self.param('rear_clearance') + extra)
        close = front | rear
        return bool(np.any(close & on_left)), bool(np.any(close & ~on_left))

    def publish(self, steering, speed, scan_msg):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = scan_msg.header.stamp
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = float(speed)
        self.drive_publisher.publish(drive_msg)


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Handle Ctrl+C and termination here instead of in rclpy, so the node can still publish
    # a stop command on the way out. Otherwise the simulator keeps applying the last speed
    # and steering and the car drives off on its own.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _interrupt)

    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDriveStamped()
        for _ in range(3):
            node.drive_publisher.publish(stop)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
