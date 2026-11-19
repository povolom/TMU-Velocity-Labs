"""Lab 4 on levine_obs: Levine with 13 obstacles, counter-clockwise from the east hallway.

    ros2 launch gap_follow levine_obs_launch.py sim:=true  # node + simulator + RViz
    ros2 launch gap_follow levine_obs_launch.py            # node only

The gaps between obstacles are 0.45 to 0.7 m for a 0.31 m car, too tight for plain follow the
gap at speed, so this map runs the obstacle_avoidance node: it knows the track, plans a path
around what the LiDAR sees and checks every command before sending it. To run the follow the
gap node here instead:

    ros2 launch gap_follow levine_obs_launch.py sim:=true driver:=reactive_node

The node follows a racing line planned offline from this map (config/levine_obs_line.csv,
made by `ros2 run gap_follow make_racing_line levine_obs`), the way the ForzaETH and UNICORN
stacks drive from their global planners, and only routes off it round something that isn't
on the map.

The top speed can be set for a run without editing the node:

    ros2 launch gap_follow levine_obs_launch.py sim:=true max_speed:=5.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Tested at max_speed 2 to 8 (two laps, with up to 30 ms of extra lag); past about 5 the
# obstacles and corners set the pace, about 35 s a lap.
PARAMETERS = {
    'obstacle_avoidance': {
        'max_speed': 8.0,
        'racing_line': os.path.join(get_package_share_directory('gap_follow'), 'config',
                                    'levine_obs_line.csv'),
    },
    'reactive_node': {'max_speed': 1.5},
}
SPEEDS = ('max_speed',)


def _node(context):
    driver = LaunchConfiguration('driver').perform(context)
    params = dict(PARAMETERS.get(driver, {}))
    params['use_sim_time'] = LaunchConfiguration('sim').perform(context) == 'true'
    for name in SPEEDS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            params[name] = float(value)
    return [Node(
        package='gap_follow',
        executable=driver,
        name=driver,
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lab_sim'), 'launch', 'sim_launch.py')),
        launch_arguments={
            'map': 'levine_obs',
            'sx': '9.96', 'sy': '2.8', 'stheta': '1.5708',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('driver', default_value='obstacle_avoidance'),
        *[DeclareLaunchArgument(name, default_value='') for name in SPEEDS],
        sim,
        OpaqueFunction(function=_node),
    ])
