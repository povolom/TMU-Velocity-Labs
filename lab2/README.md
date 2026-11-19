# Lab 2: Automatic Emergency Braking

[RoboRacer Lab 2](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab2.html): a safety node that stops the car before a collision, using Time to Collision calculated from the LaserScan data. I ran the RoboRacer simulator in Docker, wrote the safety node with AI assistance (ChatGPT) and tested it in simulation.

The package is in [safety_node/](safety_node/). The node reads `/scan` and `/ego_racecar/odom`, works out the smallest instantaneous Time to Collision across the scan, and publishes a zero-speed command on `/drive` when it drops below the `ttc_threshold` parameter (1.0 s by default).

Once it has braked, it keeps braking while something is within `hold_distance` (1.0 m) ahead. Without that, the stopped car's iTTC becomes infinite, braking lets go, and the driver's next command creeps it into the wall.

Tested in the simulator by driving at 3 m/s straight at a wall: it brakes at iTTC 0.96 s and holds the car about 1.2 m from the wall.

## Run it

In VS Code (connected to WSL), **Terminal → Run Task…** and run, in order:

1. **Lab 2 - 1. Simulator**: the simulator and RViz.
2. **Lab 2 - 2. Safety node**: the emergency brake.
3. **Lab 2 - 3. Keyboard driving**: drive with the keyboard (`i` forward, `,` back, `k` stop, `j`/`l` to turn) and head for a wall.

If the car does hit a wall, the simulator puts it back on the start line.
