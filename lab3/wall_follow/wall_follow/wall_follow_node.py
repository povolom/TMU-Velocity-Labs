import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    """PID wall follower. Follows the left wall by default (counter-clockwise laps)."""

    def __init__(self):
        super().__init__('wall_follow_node')

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

        # Speed for each steering band (the lab's table). Change these to go faster or slower.
        self.declare_parameter('straight_speed', 1.5)   # under 10 degrees
        self.declare_parameter('medium_speed', 1.0)     # 10 to 20 degrees
        self.declare_parameter('slow_speed', 0.5)       # over 20 degrees

        # Safety limits, so any speed above stays drivable. The PID gains and lookahead are
        # tuned at gain_ref_speed; faster than that, the lookahead grows and the gains shrink.
        self.declare_parameter('gain_ref_speed', 1.5)
        self.declare_parameter('lookahead_time', 0.6)    # look at least this many seconds ahead
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_lateral_accel', 4.0) # m/s^2 of grip in a turn
        self.declare_parameter('max_decel', 3.0)         # m/s^2 of braking to plan with
        self.declare_parameter('corner_speed', 1.5)      # speed to be down to at a wall ahead
        self.declare_parameter('corner_margin', 1.5)     # ...this far before reaching it

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.speed = 0.0

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
        lookahead = max(self.get_parameter('lookahead').value,
                        self.get_parameter('lookahead_time').value * self.speed)
        desired = self.get_parameter('desired_distance').value
        sign = 1.0 if self.get_parameter('side').value == 'left' else -1.0

        # b is perpendicular to the car, a is theta ahead of b, both on the wall side.
        b = self.get_range(scan_msg, sign * math.pi / 2)
        a = self.get_range(scan_msg, sign * (math.pi / 2 - theta))

        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        dist_now = b * math.cos(alpha)
        dist_ahead = dist_now + lookahead * math.sin(alpha)

        front = self.get_range(scan_msg, 0.0)
        doorway = (
            b > desired + self.get_parameter('doorway_side_jump').value
            and front > self.get_parameter('corner_front_distance').value
        )

        return desired - dist_ahead, doorway

    def pid_control(self, error, now):
        # The same steering angle turns the car harder the faster it goes, so scale the
        # gains down above the speed they were tuned at.
        scale = min(1.0, self.get_parameter('gain_ref_speed').value / max(self.speed, 1e-3))
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

    def speed_limit(self, scan_msg, steering):
        """Fastest safe speed right now, whatever the speed table asks for.

        Turning: a turn of radius r holds at most sqrt(max_lateral_accel * r) before the
        car slides, and r = wheelbase / tan(steering).
        Braking: with a wall d metres ahead, the car must still be able to slow to
        corner_speed by corner_margin before it.
        """
        limit = math.inf
        if abs(steering) > 1e-3:
            radius = self.get_parameter('wheelbase').value / math.tan(abs(steering))
            limit = math.sqrt(self.get_parameter('max_lateral_accel').value * radius)

        ahead = min(self.get_range(scan_msg, math.radians(a)) for a in (-10.0, 0.0, 10.0))
        room = max(0.0, ahead - self.get_parameter('corner_margin').value)
        corner_speed = self.get_parameter('corner_speed').value
        limit = min(limit, math.sqrt(corner_speed ** 2
                                     + 2.0 * self.get_parameter('max_decel').value * room))
        return limit

    def scan_callback(self, scan_msg):
        now = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        error, doorway = self.get_error(scan_msg)

        if doorway:
            # Drive straight past the doorway and don't let it wind up the PID.
            steering = 0.0
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_time = now
        else:
            u = self.pid_control(error, now)
            # Too close to the left wall (positive error) means steer right (negative angle).
            sign = 1.0 if self.get_parameter('side').value == 'left' else -1.0
            steering = -sign * u

        max_steering = self.get_parameter('max_steering').value
        steering = max(-max_steering, min(max_steering, steering))

        steering_deg = abs(math.degrees(steering))
        if steering_deg < 10.0:
            speed = self.get_parameter('straight_speed').value
        elif steering_deg < 20.0:
            speed = self.get_parameter('medium_speed').value
        else:
            speed = self.get_parameter('slow_speed').value

        speed = min(speed, self.speed_limit(scan_msg, steering))
        self.speed = speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = scan_msg.header.stamp
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    node = WallFollow()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
