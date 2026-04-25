"""Launch file for the Cartesian waypoint tracking demo."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("prismatic_7dof_sim")

    output_dir = LaunchConfiguration("output_dir")

    # Launch simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sim.launch.py")
        ),
        launch_arguments={"use_rviz": "true"}.items(),
    )

    # Cartesian waypoint tracker (delayed start)
    waypoints_yaml = os.path.join(pkg_share, "config", "waypoints.yaml")
    cartesian_tracker = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="prismatic_7dof_sim",
                executable="cartesian_waypoint_tracker",
                name="cartesian_waypoint_tracker",
                output="screen",
                parameters=[
                    {"output_dir": output_dir},
                    {"waypoints_file": waypoints_yaml},
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="/tmp/prismatic_plots",
            description="Directory for output plot images",
        ),
        sim_launch,
        cartesian_tracker,
    ])
