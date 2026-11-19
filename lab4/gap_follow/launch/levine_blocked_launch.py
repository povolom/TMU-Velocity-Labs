"""Lab 4 on levine_blocked: the empty Levine loop, counter-clockwise from the south hallway.

    ros2 launch gap_follow levine_blocked_launch.py sim:=true  # node + simulator + RViz
    ros2 launch gap_follow levine_blocked_launch.py            # node only, as the lab's grader runs it

The top speed can be set for a run without editing the node:

    ros2 launch gap_follow levine_blocked_launch.py sim:=true max_speed:=10.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMETERS = {}
SPEEDS = ('max_speed',)


def _node(context):
    # Only pass the speeds given on the command line, so PARAMETERS and the node's own
    # defaults apply otherwise.
    params = dict(PARAMETERS)
    params['use_sim_time'] = LaunchConfiguration('sim').perform(context) == 'true'
    for name in SPEEDS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            params[name] = float(value)
    return [Node(
        package='gap_follow',
        executable='reactive_node',
        name='reactive_node',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lab_sim'), 'launch', 'sim_launch.py')),
        launch_arguments={
            'map': 'levine_blocked',
            'sx': '-12.0', 'sy': '0.0', 'stheta': '0.0',
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        *[DeclareLaunchArgument(name, default_value='') for name in SPEEDS],
        sim,
        OpaqueFunction(function=_node),
    ])
