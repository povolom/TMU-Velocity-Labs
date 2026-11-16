"""Lab 4 on levine_blocked: the empty Levine loop, counter-clockwise from the south hallway.

    ros2 launch gap_follow levine_blocked_launch.py sim:=true  # node + simulator + RViz
    ros2 launch gap_follow levine_blocked_launch.py            # node only, as the lab's grader runs it
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMETERS = {}


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

    reactive_node = Node(
        package='gap_follow',
        executable='reactive_node',
        name='reactive_node',
        output='screen',
        parameters=[PARAMETERS, {'use_sim_time': LaunchConfiguration('sim')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        reactive_node,
    ])
