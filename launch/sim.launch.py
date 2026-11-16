"""Gazebo only: generate the site's world (warehouse or airport), start Gazebo,
spawn every robot (and, at the airport, the passengers) and bridge their
topics into ROS 2. No robot software runs, so everything sits still until
something publishes /<name>/cmd_vel.

    ros2 launch confluence_sim sim.launch.py
    ros2 launch confluence_sim sim.launch.py site:=airport headless:=true
(The full system is confluence.launch.py, which includes this file.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from confluence import site
from confluence_sim.fleet_config import guests, passengers, person_sdf, robot_sdf, robots
from confluence_sim.world_gen import build_world


def _setup(context):
    # Build the world fresh on every launch -> the environment is created automatically.
    site.load(LaunchConfiguration("site").perform(context))
    seed = int(LaunchConfiguration("seed").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    world = os.path.join(os.path.expanduser("~"), ".ros", f"confluence_{site.NAME}.sdf")
    os.makedirs(os.path.dirname(world), exist_ok=True)
    with open(world, "w") as f:
        f.write(build_world(seed))
    print(f"[confluence_sim] generated {site.TITLE} world -> {world} (seed {seed})")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {'-s ' if headless else ''}{world}"}.items(),
    )

    spawners = []
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    things = [(r["name"], r["spawn"], robot_sdf(r), True) for r in robots()]
    things += [(p["name"], p["spawn"], person_sdf(p), False) for p in passengers() + guests()]
    for n, (x, y), sdf, has_lidar in things:
        spawners.append(Node(
            package="ros_gz_sim", executable="create", name=f"spawn_{n}", output="screen",
            arguments=["-world", site.NAME, "-name", n, "-string", sdf,
                       "-x", str(x), "-y", str(y), "-z", "0.02"],
        ))
        bridge_args += [
            f"/{n}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/{n}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ]
        if has_lidar:
            bridge_args.append(f"/{n}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="fleet_bridge",
                  output="screen", arguments=bridge_args)

    return [
        gz,
        TimerAction(period=4.0, actions=spawners),   # let Gazebo load the world first
        bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("site", default_value="warehouse", description="warehouse | airport"),
        SetEnvironmentVariable("CONFLUENCE_SITE", LaunchConfiguration("site")),
        DeclareLaunchArgument("seed", default_value="7", description="Random seed for the layout details"),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo without its GUI"),
        OpaqueFunction(function=_setup),
    ])
