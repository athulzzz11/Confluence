"""Protocol adapters. Add a new vendor protocol by subclassing RobotAdapter
and registering it here."""


def make_adapter(cfg, ros_bus=None):
    proto = cfg["protocol"]
    if proto == "ros2":
        from confluence.adapters.ros2 import Ros2Adapter
        if ros_bus is None:
            raise RuntimeError(f"{cfg['id']}: protocol ros2 needs a ROS 2 bus")
        return Ros2Adapter(cfg, ros_bus)
    if proto == "rest":
        from confluence.adapters.rest import RestAdapter
        return RestAdapter(cfg)
    if proto == "tcp":
        from confluence.adapters.tcp import TcpAdapter
        return TcpAdapter(cfg)
    if proto == "vda5050":
        from confluence.adapters.vda5050 import Vda5050Adapter
        return Vda5050Adapter(cfg)
    raise ValueError(f"unknown protocol '{proto}' for robot {cfg['id']}")
