# TMU Velocity labs

My software onboarding labs for TMU Velocity, Toronto Metropolitan University's autonomous vehicles design team. I'm on the software team for the RoboRacer (F1TENTH) autonomous racing program, which runs on ROS 2 and Ubuntu.

Built by [Marcantonio Povolo](https://marcantoniopovolo.com), a Computer Engineering student at TMU.

## What I did

- Completed software onboarding Labs 1 and 2 in ROS 2 on Ubuntu, resolving ROS distribution and package conflicts along the way.
- Set up the RoboRacer simulator in Docker and wrote an emergency-braking safety node with AI assistance (ChatGPT), then tested it in simulation.

These are Labs 1 and 2 of the [RoboRacer course](https://f1tenth-coursekit.readthedocs.io/en/latest/) (formerly F1TENTH), which the team uses for software onboarding:

| Lab | What it is | Folder |
|---|---|---|
| Lab 1: Introduction to ROS 2 | ROS 2 basics: workspaces, packages and nodes that publish and subscribe to messages | [lab1/](lab1/) |
| Lab 2: Automatic Emergency Braking | A safety node that stops the car before a collision, using Time to Collision calculated from the LaserScan data in the RoboRacer simulator | [lab2/](lab2/) |

## Layout

| Path | What it is |
|---|---|
| `lab1/`, `lab2/` | One folder per lab: the ROS 2 package source and a short note on what I did |

ROS 2's generated folders (`build/`, `install/`, `log/`) aren't committed; `colcon build` recreates them.
