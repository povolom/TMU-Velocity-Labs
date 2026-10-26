# Lab 2: Automatic Emergency Braking

[RoboRacer Lab 2](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab2.html): a safety node that stops the car before a collision, using Time to Collision calculated from the LaserScan data. I ran the RoboRacer simulator in Docker, wrote the safety node with AI assistance (ChatGPT) and tested it in simulation.

The package is in [safety_node/](safety_node/). The node reads `/scan` and `/ego_racecar/odom`, works out the smallest Time to Collision across the scan, and publishes a zero-speed command on `/drive` when it drops below the `ttc_threshold` parameter (1.0 s by default).

To run it in the simulator, mount the package into the `sim` container in the `f1tenth_gym_ros` `docker-compose.yml`:

```yaml
volumes:
  - /home/<you>/safety_node:/sim_ws/src/safety_node
```
