"""Lab 4 on levine_obs: Levine with 13 obstacles, counter-clockwise from the east hallway.

    ros2 launch gap_follow levine_obs_launch.py sim:=true  # node + simulator + RViz
    ros2 launch gap_follow levine_obs_launch.py            # node only, as the lab's grader runs it
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Slower than the defaults: the gaps between obstacles are 0.45 to 0.7 m for a 0.31 m car.
# Tested at these speeds; at 2.0 m/s or more the car takes the wrong line at the L-shaped
# pocket and crashes there.
PARAMETERS = {
    'max_speed': 1.5,
}


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
