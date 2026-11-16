# Lab 3: Wall Following

[RoboRacer Lab 3](https://github.com/f1tenth/f1tenth_lab3_template): drive laps by holding a set distance from a wall, using a PID controller on the LiDAR data.

![Lab 3 wall follower driving a lap of levine_blocked](media/wall_follow_levine_blocked.gif)

*Replay of a simulator run, drawn from the car's recorded positions (2x speed). One counter-clockwise lap of `levine_blocked` in 43.6 s without touching a wall.*

## How it works

The package is in [wall_follow/](wall_follow/). The node `wall_follow_node` reads `/scan` and publishes `/drive`. It follows the **left** wall, the inner wall on a counter-clockwise lap.

1. Take two beams on the left: `b` at 90° and `a` at 90° − θ (θ = 50°).
2. Angle to the wall: `α = atan((a·cos θ − b) / (a·sin θ))`.
3. Distance now: `D_t = b·cos α`. Distance one lookahead ahead: `D_t+1 = D_t + L·sin α` (L = 1.0 m).
4. Error = desired distance (1.0 m) − `D_t+1`, fed through PID (Kp = 1.0, Ki = 0, Kd = 0.1) to get the steering angle.
5. Speed from the steering angle: 1.5 m/s under 10°, 1.0 m/s under 20°, 0.5 m/s otherwise.

A doorway can look like a corner to two beams. The node tells them apart the way the lab suggests: if the left wall disappears but there is open space more than 6 m ahead, it is a doorway, so the car drives straight past it instead of turning in.

All values are ROS parameters with these defaults (`kp`, `ki`, `kd`, `desired_distance`, `lookahead`, `theta_deg`, `side`).

## Run it

In VS Code (connected to WSL), **Terminal → Run Task… → Lab 3: wall follow (simulator)**, or:

```bash
ros2 launch wall_follow wall_follow_launch.py sim:=true  # simulator on levine_blocked + RViz + the node
ros2 run wall_follow wall_follow_node                    # just the node, e.g. on the car
```

The tests in [wall_follow/test/](wall_follow/test/) check the node like the lab's grader does, on synthetic corridor scans: too close to either wall, or turned toward either wall, it must steer away. Run them with `python3 -m pytest lab3/wall_follow/test`.

Written with AI assistance (Claude Code) and tested in the RoboRacer simulator.
