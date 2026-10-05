# TMU Velocity labs

My software onboarding labs for TMU Velocity, Toronto Metropolitan University's autonomous vehicles design team. I'm on the software team for the RoboRacer (F1TENTH) autonomous racing program, which runs on ROS 2 and Ubuntu.

Built by [Marcantonio Povolo](https://marcantoniopovolo.com), a Computer Engineering student at TMU.

## What I did

- Completed software onboarding Labs 1 and 2 in ROS 2 on Ubuntu, resolving ROS distribution and package conflicts along the way.
- Set up the RoboRacer simulator in Docker and wrote an emergency-braking safety node with AI assistance (ChatGPT), then tested it in simulation.

| Lab | Folder |
|---|---|
| Lab 1 | [lab1/](lab1/) |
| Lab 2 | [lab2/](lab2/) |

## Layout

| Path | What it is |
|---|---|
| `lab1/`, `lab2/` | One folder per lab: the ROS 2 package source and a short note on what I did |

ROS 2's generated folders (`build/`, `install/`, `log/`) aren't committed; `colcon build` recreates them.
