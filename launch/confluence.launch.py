"""Full system: Gazebo site (warehouse or airport terminal) + 4 robots from 4
(simulated) vendors, each with its own interface, + Confluence controlling them.

    ros2 launch confluence_sim confluence.launch.py
    ros2 launch confluence_sim confluence.launch.py site:=airport
    ros2 launch confluence_sim confluence.launch.py site:=airport headless:=true dashboard_port:=8080

Start-up order:
  t=0 s  Gazebo + world (generated)            sim.launch.py
  t=4 s  robots spawned, ros_gz_bridge up
  t=7 s  vendor robot software (ROS 2 / REST / TCP / VDA 5050 APIs)
  t=9 s  Confluence (ROS 2 node + dashboard on http://localhost:8080)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from confluence.core import DEFAULT_CONFIG


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("confluence_sim"), "launch", "sim.launch.py")),
        launch_arguments={"site": LaunchConfiguration("site"),
                          "seed": LaunchConfiguration("seed"),
                          "headless": LaunchConfiguration("headless")}.items(),
    )
    vendors = Node(package="confluence_sim", executable="vendor_robots", name="vendor_robots",
                   output="screen", arguments=["--backend", "gazebo"])
    confluence = Node(package="confluence_sim", executable="confluence", name="confluence", output="screen",
                      parameters=[{"config": LaunchConfiguration("config"),
                                   "dashboard_port": LaunchConfiguration("dashboard_port")}])
    return LaunchDescription([
        DeclareLaunchArgument("site", default_value="warehouse", description="warehouse | airport"),
        SetEnvironmentVariable("CONFLUENCE_SITE", LaunchConfiguration("site")),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("config", default_value=DEFAULT_CONFIG,
                              description="fleet config path; auto = confluence/config/fleet_<site>.yaml"),
        DeclareLaunchArgument("dashboard_port", default_value="8080"),
        sim,
        TimerAction(period=7.0, actions=[vendors]),
        TimerAction(period=9.0, actions=[confluence]),
    ])
