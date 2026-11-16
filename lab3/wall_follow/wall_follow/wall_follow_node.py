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

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

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
        lookahead = self.get_parameter('lookahead').value
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
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        dt = 0.0 if self.prev_time is None else now - self.prev_time
        if dt > 0.0:
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        self.prev_error = error
        self.prev_time = now

        return kp * error + ki * self.integral + kd * derivative

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
            speed = 1.5
        elif steering_deg < 20.0:
            speed = 1.0
        else:
            speed = 0.5

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
