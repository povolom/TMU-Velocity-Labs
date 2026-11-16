import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    """Follow the gap: drive toward the gap the car fits through that leads furthest forward."""

    def __init__(self):
        super().__init__('reactive_node')

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
        self.declare_parameter('slow_down_distance', 1.5) # slow speed if anything is this close ahead
        self.declare_parameter('fast_speed', 2.0)
        self.declare_parameter('medium_speed', 1.5)
        self.declare_parameter('slow_speed', 1.0)
        # Safety limits, so any speeds above stay drivable.
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_lateral_accel', 4.0) # m/s^2 of grip in a turn
        self.declare_parameter('max_decel', 3.0)         # m/s^2 of braking to plan with
        self.declare_parameter('corner_speed', 1.5)      # speed to be down to at a wall ahead
        self.declare_parameter('corner_margin', 1.0)     # ...this far before reaching it

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
        half_width = self.param('car_half_width')
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

        steering_deg = abs(np.degrees(steering))
        if steering_deg < 10.0:
            speed = self.param('fast_speed')
        elif steering_deg < 20.0:
            speed = self.param('medium_speed')
        else:
            speed = self.param('slow_speed')

        ahead = ranges[np.abs(angles) < np.radians(15.0)]
        if end < start or (ahead.size and ahead.min() < self.param('slow_down_distance')):
            speed = min(speed, self.param('slow_speed'))

        speed = min(speed, self.speed_limit(steering, ahead.min() if ahead.size else np.inf))
        self.publish(steering, speed, scan_msg)

    def speed_limit(self, steering, ahead):
        """Fastest safe speed right now, whatever the speed settings ask for.

        Turning: a turn of radius r holds at most sqrt(max_lateral_accel * r) before the
        car slides, and r = wheelbase / tan(steering).
        Braking: with something `ahead` metres in front, the car must still be able to slow
        to corner_speed by corner_margin before it.
        """
        limit = np.inf
        if abs(steering) > 1e-3:
            radius = self.param('wheelbase') / np.tan(abs(steering))
            limit = np.sqrt(self.param('max_lateral_accel') * radius)

        room = max(0.0, ahead - self.param('corner_margin'))
        return min(limit, np.sqrt(self.param('corner_speed') ** 2
                                  + 2.0 * self.param('max_decel') * room))

    def side_blocked(self, scan_msg):
        """Whether something sits right beside the car's body on the left / right.

        Looks alongside the car's body (the LiDAR sits near the front bumper): out to
        side_clearance beside the front half, where a corner the car is about to clip shows up
        first, and out to rear_clearance beside the rear half. The rear zone is narrower so an
        obstacle the car has just passed doesn't block the next turn. Nothing ahead of the
        bumper counts, or the check would block steering around an obstacle in front.
        """
        ranges = np.array(scan_msg.ranges, dtype=float)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        ok = np.isfinite(ranges) & (ranges > 0.0)
        x = ranges[ok] * np.cos(angles[ok])
        y = np.abs(ranges[ok] * np.sin(angles[ok]))
        on_left = angles[ok] > 0.0

        front = (x >= -0.15) & (x < 0.15) & (y < self.param('side_clearance'))
        rear = (x > -0.4) & (x < -0.15) & (y < self.param('rear_clearance'))
        close = front | rear
        return bool(np.any(close & on_left)), bool(np.any(close & ~on_left))

    def publish(self, steering, speed, scan_msg):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = scan_msg.header.stamp
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = float(speed)
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ReactiveFollowGap()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
