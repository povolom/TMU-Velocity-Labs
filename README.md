# TMU Velocity labs

My software onboarding labs for TMU Velocity, Toronto Metropolitan University's autonomous vehicles design team. I'm on the software team for the RoboRacer (F1TENTH) autonomous racing program, which runs on ROS 2 and Ubuntu.

Built by [Marcantonio Povolo](https://marcantoniopovolo.com), a Computer Engineering student at TMU.

## What I did

- Completed software onboarding Labs 1 and 2 in ROS 2 on Ubuntu, resolving ROS distribution and package conflicts along the way.
- Set up the RoboRacer simulator in Docker and wrote an emergency-braking safety node with AI assistance (ChatGPT), then tested it in simulation.
- Labs 3 and 4 (wall following and follow the gap) were written with AI assistance (Claude Code) and tested in the RoboRacer simulator running natively on Ubuntu, without Docker.

These are Labs 1 to 4 of the [RoboRacer course](https://f1tenth-coursekit.readthedocs.io/en/latest/) (formerly F1TENTH), which the team uses for software onboarding:

| Lab | What it is | Folder |
|---|---|---|
| Lab 1: Introduction to ROS 2 | ROS 2 basics: workspaces, packages and nodes that publish and subscribe to messages | [lab1/](lab1/) |
| Lab 2: Automatic Emergency Braking | A safety node that stops the car before a collision, using Time to Collision calculated from the LaserScan data in the RoboRacer simulator | [lab2/](lab2/) |
| Lab 3: Wall Following | A PID controller that holds a set distance from the inner wall to drive laps | [lab3/](lab3/) |
| Lab 4: Follow the Gap | A reactive planner that drives toward the free space it fits through, on an empty track and one full of obstacles | [lab4/](lab4/) |

![Lab 4 follow the gap, 3 laps of levine_blocked](lab4/media/gap_follow_levine_blocked.gif)

## Layout

| Path | What it is |
|---|---|
| `lab1/` to `lab4/` | One folder per lab: the ROS 2 package source and a short note on what I did |
| `sim/lab_sim/` | Launches the RoboRacer simulator on a chosen map and start pose; the Lab 3 and 4 launch files use it |

ROS 2's generated folders (`build/`, `install/`, `log/`) aren't committed; `colcon build` recreates them.

## Build and run

On Ubuntu 22.04 with ROS 2 Humble and [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) in `~/f1tenth_gym_ros` (with the `levine_blocked` and `levine_obs` maps in its `maps/` folder), link the packages into a colcon workspace and build:

```bash
mkdir -p ~/velocity_ws/src
cd ~/TMU-Velocity-Labs
ln -s $PWD/lab1/lab1_pkg $PWD/lab2/safety_node $PWD/lab3/wall_follow $PWD/lab4/gap_follow $PWD/sim/lab_sim ~/f1tenth_gym_ros ~/velocity_ws/src/
cd ~/velocity_ws && colcon build --symlink-install && source install/setup.bash
```

```bash
ros2 launch lab1_pkg lab1_launch.py                # Lab 1: talker + relay
ros2 run safety_node safety_node                   # Lab 2: run alongside the RoboRacer simulator
ros2 launch wall_follow wall_follow_launch.py      # Lab 3: simulator + RViz + wall follower
ros2 launch gap_follow levine_blocked_launch.py    # Lab 4: simulator + RViz + follow the gap
ros2 launch gap_follow levine_obs_launch.py        # Lab 4 on the obstacle map
```

The simulator runs on sim time from `lab_sim`'s steady clock, because WSL's system clock can jump by minutes and freeze ROS timers.
