# Lab 1: Introduction to ROS 2

[RoboRacer Lab 1](https://f1tenth-coursekit.readthedocs.io/en/latest/) covers ROS 2 basics: workspaces, packages and nodes that publish and subscribe to messages. I set it up on Ubuntu and resolved ROS distribution and package conflicts along the way.

The package is in [lab1_pkg/](lab1_pkg/): a `talker` node and a `relay` node (in `scripts/`), started together by `launch/lab1_launch.py`.

```bash
colcon build --packages-select lab1_pkg --symlink-install
ros2 launch lab1_pkg lab1_launch.py
```
