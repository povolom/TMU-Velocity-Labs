"""Start the RoboRacer simulator (f1tenth_gym_ros) on a chosen map and start pose.

Same nodes as f1tenth_gym_ros's gym_bridge_launch.py, but the map and start pose
are launch arguments instead of edits to sim.yaml. Everything runs on sim time from
lab_sim's steady_clock, so WSL clock jumps don't freeze the simulator.

    ros2 launch lab_sim sim_launch.py map:=levine_blocked sx:=-12.0 sy:=0.0 stheta:=0.0
"""

import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _other_simulator_running():
    """Whether a gym_bridge is already running on this ROS domain.

    Two simulators on one domain share topics and the map server's name, which mixes up
    the car and leaves the new map unloaded.
    """
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                if b'gym_bridge' not in f.read():
                    continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = dict(v.split(b'=', 1) for v in f.read().split(b'\0') if b'=' in v)
        except OSError:
            continue
        if env.get(b'ROS_DOMAIN_ID', b'0').decode() == domain:
            return True
    return False


# Lab nodes that drive the car. One left over from an earlier run (its launch gone) would
# keep sending commands and fight the new run's driver.
DRIVERS = (b'/lib/gap_follow/', b'/lib/wall_follow/', b'/lib/safety_node/', b'/lib/lab_sim/')


def _stop_leftover_drivers():
    """Stop lab nodes on this ROS domain that no running launch or ros2 run owns."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmdline = f.read()
            if not any(d in cmdline for d in DRIVERS):
                continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = dict(v.split(b'=', 1) for v in f.read().split(b'\0') if b'=' in v)
            if env.get(b'ROS_DOMAIN_ID', b'0').decode() != domain:
                continue
            with open(f'/proc/{pid}/stat', 'rb') as f:
                parent = f.read().rsplit(b')', 1)[1].split()[1].decode()
            with open(f'/proc/{parent}/cmdline', 'rb') as f:
                if b'ros2' in f.read():
                    continue        # still part of a running launch
        except (OSError, IndexError):
            continue
        print(f'[lab_sim] stopping a lab node left over from an earlier run (pid {pid})')
        os.kill(int(pid), signal.SIGKILL)


def _nodes(context):
    _stop_leftover_drivers()
    if _other_simulator_running():
        raise RuntimeError(
            'Another simulator is already running. Stop it first (trash-can icon on its '
            'VS Code terminal, or Ctrl+C), then start this one again.')

    gym_share = get_package_share_directory('f1tenth_gym_ros')
    map_dir = os.path.expanduser(LaunchConfiguration('map_dir').perform(context))
    map_path = os.path.join(map_dir, LaunchConfiguration('map').perform(context))

    bridge = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[
            os.path.join(gym_share, 'config', 'sim.yaml'),
            {
                'use_sim_time': True,
                'map_path': map_path,
                'num_agent': 1,
                'sx': float(LaunchConfiguration('sx').perform(context)),
                'sy': float(LaunchConfiguration('sy').perform(context)),
                'stheta': float(LaunchConfiguration('stheta').perform(context)),
            },
        ],
    )
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': True}],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )
    robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'use_sim_time': True},
                    {'robot_description': Command(
                        ['xacro ', os.path.join(gym_share, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('lab_sim'), 'rviz', 'lab_sim.rviz')],
        parameters=[{'use_sim_time': True}],
        # Software OpenGL: the setup verified to draw the map and car on WSL.
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    clock = Node(package='lab_sim', executable='steady_clock', name='steady_clock')
    crash_reset = Node(
        package='lab_sim',
        executable='crash_reset',
        name='crash_reset',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'sx': float(LaunchConfiguration('sx').perform(context)),
            'sy': float(LaunchConfiguration('sy').perform(context)),
            'stheta': float(LaunchConfiguration('stheta').perform(context)),
        }],
        condition=IfCondition(LaunchConfiguration('reset_on_crash')),
    )
    return [clock, bridge, map_server, lifecycle, robot_publisher, rviz, crash_reset]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='levine_blocked',
                              description='Map name in map_dir, without extension'),
        DeclareLaunchArgument('map_dir', default_value='~/f1tenth_gym_ros/maps'),
        DeclareLaunchArgument('sx', default_value='-12.0'),
        DeclareLaunchArgument('sy', default_value='0.0'),
        DeclareLaunchArgument('stheta', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('reset_on_crash', default_value='true',
                              description='Put the car back on the start line after a crash'),
        OpaqueFunction(function=_nodes),
    ])
