"""Lab 3: wall follower, optionally with the simulator on levine_blocked.

    ros2 launch wall_follow wall_follow_launch.py sim:=true  # node + simulator + RViz
    ros2 launch wall_follow wall_follow_launch.py            # node only (car, or a simulator already running)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    wall_follow = Node(
        package='wall_follow',
        executable='wall_follow_node',
        name='wall_follow_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('sim')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        wall_follow,
    ])
