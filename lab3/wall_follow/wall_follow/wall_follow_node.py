import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# How fast the car can turn and brake (the RoboRacer car in the simulator). The speed is
# always kept within these, so max_speed can be set to anything.
WHEELBASE = 0.33          # m
LATERAL_ACCEL = 7.0       # m/s^2 of grip in a turn
BRAKE_DECEL = 5.0         # m/s^2 of braking
CORNER_SPEED = 2.0        # m/s to be down to...
CORNER_MARGIN = 1.0       # ...this many metres before a wall ahead
SPEED_UP = 5.0            # m/s^2: how fast the car may gain speed

# The PID gains and lookahead are tuned at 1.5 m/s. Faster than SMOOTH_ABOVE, the lookahead
# grows, the gains shrink and steering is smoothed, so the car doesn't twitch.
TUNED_SPEED = 1.5         # m/s
SMOOTH_ABOVE = 2.5        # m/s
LOOKAHEAD_TIME = 0.6      # s of travel to look ahead, at least
STEERING_BLEND = 0.35     # share of each new steering command when smoothing


class WallFollow(Node):
    """PID wall follower. Follows the left wall by default (counter-clockwise laps)."""

    def __init__(self):
        super().__init__('wall_follow_node')

        # Top speed, on straights. Set it to anything: the car slows for turns and walls by
        # itself. At 1.5 the speeds are exactly the lab's table (1.5 / 1.0 / 0.5 m/s).
        self.declare_parameter('max_speed', 1.5)

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('theta_deg', 50.0)
        self.declare_parameter('side', 'left')
        # A wall that disappears with open space ahead is a doorway, not a corner.
        self.declare_parameter('doorway_side_jump', 1.5)
        self.declare_parameter('corner_front_distance', 6.0)
        self.declare_parameter('max_steering', 0.4189)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.speed = 0.0
        self.steering = 0.0

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

    def get_range(self, scan_msg, angle):
        """Range of the beam at `angle` (radians, 0 is straight ahead, left is positive).

        NaN and inf beams are skipped by averaging the finite beams next to it.
        """
        i = round((angle - scan_msg.angle_min) / scan_msg.angle_increment)
        i = max(0, min(i, len(scan_msg.ranges) - 1))

        nearby = []
        for j in range(i - 2, i + 3):
            if 0 <= j < len(scan_msg.ranges):
                r = scan_msg.ranges[j]
                if math.isfinite(r) and scan_msg.range_min <= r <= scan_msg.range_max:
                    nearby.append(r)

        if not nearby:
            return scan_msg.range_max
        return sum(nearby) / len(nearby)

    def get_error(self, scan_msg):
        """Error between the desired wall distance and the predicted distance one lookahead ahead."""
        theta = math.radians(self.get_parameter('theta_deg').value)
        lookahead = max(self.get_parameter('lookahead').value, LOOKAHEAD_TIME * self.speed)
        desired = self.get_parameter('desired_distance').value
        sign = 1.0 if self.get_parameter('side').value == 'left' else -1.0

        # b is perpendicular to the car, a is theta ahead of b, both on the wall side.
        b = self.get_range(scan_msg, sign * math.pi / 2)
        a = self.get_range(scan_msg, sign * (math.pi / 2 - theta))

        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        dist_now = b * math.cos(alpha)
        dist_ahead = dist_now + lookahead * math.sin(alpha)

        # When the wall beside the car ends, it is either a doorway (open space ahead) or a
        # corner (a wall ahead). The formula above can't tell and can even steer away from the
        # wall at a corner, so both cases are reported separately.
        front = self.get_range(scan_msg, 0.0)
        wall_ended = b > desired + self.get_parameter('doorway_side_jump').value
        open_ahead = front > self.get_parameter('corner_front_distance').value

        return desired - dist_ahead, wall_ended and open_ahead, wall_ended and not open_ahead

    def pid_control(self, error, now):
        # The same steering angle turns the car harder the faster it goes, so scale the
        # gains down above the speed they were tuned at.
        scale = min(1.0, TUNED_SPEED / max(self.speed, 1e-3))
        kp = self.get_parameter('kp').value * scale
        ki = self.get_parameter('ki').value * scale
        kd = self.get_parameter('kd').value * scale

        dt = 0.0 if self.prev_time is None else now - self.prev_time
        if dt > 0.0:
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        self.prev_error = error
        self.prev_time = now

        return kp * error + ki * self.integral + kd * derivative

    def choose_speed(self, scan_msg, steering, dt):
        """The lab's speed table scaled to max_speed, kept within what the car can turn and
        brake for."""
        max_speed = self.get_parameter('max_speed').value
        steering_deg = abs(math.degrees(steering))
        if steering_deg < 10.0:
            speed = max_speed
        elif steering_deg < 20.0:
            speed = max_speed * 2.0 / 3.0
        else:
            speed = max_speed / 3.0

        # A turn of radius r = wheelbase / tan(steering) holds at most sqrt(grip * r).
        if abs(steering) > 1e-3:
            speed = min(speed, math.sqrt(LATERAL_ACCEL * WHEELBASE / math.tan(abs(steering))))

        # Brake early enough to be down to CORNER_SPEED by CORNER_MARGIN before a wall ahead.
        ahead = min(self.get_range(scan_msg, math.radians(a)) for a in (-10.0, 0.0, 10.0))
        room = max(0.0, ahead - CORNER_MARGIN)
        speed = min(speed, math.sqrt(CORNER_SPEED ** 2 + 2.0 * BRAKE_DECEL * room))

        # Gain speed gradually at high speed; slowing down is never delayed.
        if speed > max(self.speed, SMOOTH_ABOVE) and dt > 0.0:
            speed = min(speed, max(self.speed, SMOOTH_ABOVE) + SPEED_UP * dt)
        return speed

    def scan_callback(self, scan_msg):
        now = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.prev_time is None else now - self.prev_time
        error, doorway, corner = self.get_error(scan_msg)
        sign = 1.0 if self.get_parameter('side').value == 'left' else -1.0
        max_steering = self.get_parameter('max_steering').value

        if doorway or corner:
            # Doorway: drive straight past it. Corner: turn around it, toward the wall side.
            # Either way, don't let the jump in the readings wind up the PID.
            steering = sign * max_steering if corner else 0.0
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_time = now
        else:
            u = self.pid_control(error, now)
            # Too close to the left wall (positive error) means steer right (negative angle).
            steering = -sign * u

        steering = max(-max_steering, min(max_steering, steering))

        if self.speed > SMOOTH_ABOVE:
            steering = STEERING_BLEND * steering + (1.0 - STEERING_BLEND) * self.steering
        self.steering = steering

        self.speed = self.choose_speed(scan_msg, steering, dt)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = scan_msg.header.stamp
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = self.speed
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    node = WallFollow()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
