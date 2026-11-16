"""ROS 2-native robots (e.g. Nimbus AMR-S): goal_pose / command / status topics."""
import json
import math

from confluence.adapters.base import RobotAdapter
from confluence.model import CHARGING, ERROR, IDLE, MOVING, PAUSED

MODE_MAP = {"NAVIGATING": MOVING, "IDLE": IDLE, "PAUSED": PAUSED, "BLOCKED": MOVING,
            "DOCKED_CHARGING": CHARGING, "FAULT": ERROR}


class Ros2Adapter(RobotAdapter):
    protocol = "ros2"
    supports_speed_limit = True
    nominal_hz = 10.0

    def __init__(self, cfg, bus):
        super().__init__(cfg)
        self.bus = bus
        self.ns = cfg["endpoint"]["namespace"].rstrip("/")
        self._limit = None

    def start(self):
        self.bus.subscribe(f"{self.ns}/api/status", self._on_status)

    def _on_status(self, text):
        try:
            m = json.loads(text)
        except ValueError:
            return
        p = m["pose"]
        mode = m.get("mode", "IDLE")
        self._received(x=p["x"], y=p["y"], yaw=p["yaw"],
                       battery=round(m["battery"] * 100.0, 1),           # 0..1 -> %
                       speed=m.get("linear_velocity", 0.0),
                       state=MODE_MAP.get(mode, ERROR),
                       charging=mode == "DOCKED_CHARGING",
                       safety_stop=mode == "BLOCKED",
                       errors=list(m.get("faults", [])), extra={"escort": m.get("escort")}, raw=m)

    def drive(self, start, waypoints, extend=False, action=None, action_seconds=None):
        if not self.link_up() or not waypoints:
            return False
        speed = waypoints[-1].get("speed") or 0.0
        if speed != self._limit:
            self.bus.publish(f"{self.ns}/api/speed_limit", speed)
            self._limit = speed
        w = waypoints[-1]
        yaw = math.atan2(w["y"] - start["y"], w["x"] - start["x"])
        self.bus.publish(f"{self.ns}/api/goal_pose", (w["x"], w["y"], yaw))
        self.tx_count += 1
        return True

    def _cmd(self, c):
        if not self.link_up():
            return False
        self.bus.publish(f"{self.ns}/api/command", c)
        self.tx_count += 1
        return True

    def signal(self, name, on):
        if name == "escort":
            return self._cmd("escort on" if on else "escort off")
        return False

    def cancel(self):
        return self._cmd("cancel")

    def pause(self):
        return self._cmd("pause")

    def resume(self):
        return self._cmd("resume")
