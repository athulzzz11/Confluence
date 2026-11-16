from glob import glob
from setuptools import setup

package_name = "confluence_sim"

setup(
    name=package_name,
    version="0.2.0",
    # confluence        = the orchestration platform (vendor-agnostic, ROS 2 node)
    # confluence_sim    = the Gazebo warehouse + simulated vendor robots it controls
    packages=[package_name, "confluence_sim.vendors",
              "confluence", "confluence.adapters", "confluence.dashboard", "confluence.sites"],
    package_data={"confluence": ["config/*.yaml", "dashboard/*.html"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("confluence/config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "confluence = confluence.ros_node:main",
            "confluence_lite = confluence.lite:main",
            "confluence_teleop = confluence.teleop:main",
            "vendor_robots = confluence_sim.vendors.run:main",
            "make_world = confluence_sim.world_gen:main",
            "mini_broker = confluence_sim.vendors.mini_broker:main_cli",
        ],
    },
)
