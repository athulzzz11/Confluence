"""Adapter contract. One adapter per vendor protocol; everything above this
layer only sees RobotState and these methods.

Rules every adapter follows (project constraint):
  * talk to the robot only through its published vendor interface
  * never modify robot software, never bypass its safety system
    (a robot holding itself on a safety stop stays stopped)
"""
import math
import threading
import time

from confluence.model import OFFLINE, RobotState


class RobotAdapter:
    protocol = "base"
    supports_speed_limit = False     # can Confluence cap the robot's speed?
    supports_native_actions = False  # can the robot run pick/drop itself?
    nominal_hz = 1.0                 # expected telemetry rate (for staleness)

    def __init__(self, cfg):
        self.cfg = cfg
        self.id = cfg["id"]
        self.capabilities = set(cfg.get("capabilities", []))
        self.state = RobotState(robot_id=self.id, vendor=cfg.get("vendor", ""),
                                model=cfg.get("model", ""), protocol=self.protocol)
        self.lock = threading.RLock()
        self._intervals = []
        self._last_rx = None
        self.link_down_until = 0.0   # simulated comms outage (robustness testing)
        self.stale_after = max(2.0, 4.0 / self.nominal_hz)
        self.tx_count = self.rx_count = 0

    # ------------------------------------------------ lifecycle
    def start(self):
        raise NotImplementedError

    def stop(self):
        pass

    # ------------------------------------------------ commands (uniform)
    def drive(self, start, waypoints, extend=False, action=None, action_seconds=None):
        """start: {"node_id","x","y"} the robot is at / last committed node.
        waypoints: [{"node_id","x","y","speed"}] in travel order (collinear).
        extend: True if this continues the previous drive() in the same line.
        action: None | "pick" | "drop" to perform at the last waypoint (native only).
        Returns True if the command was sent."""
        raise NotImplementedError

    def signal(self, name, on):
        """Optional vendor feature switch (e.g. a guide robot's "escort" mode).
        Returns True if the robot's interface supports it and it was sent."""
        return False

    def cancel(self):
        raise NotImplementedError

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    # ------------------------------------------------ helpers for subclasses
    def link_up(self):
        return time.time() >= self.link_down_until

    def simulate_link_loss(self, seconds):
        self.link_down_until = time.time() + seconds

    def _received(self, **fields):
        """Called by subclasses with already-normalized fields."""
        if not self.link_up():
            return
        now = time.time()
        with self.lock:
            if self._last_rx is not None:
                self._intervals = (self._intervals + [now - self._last_rx])[-20:]
                avg = sum(self._intervals) / len(self._intervals)
                self.state.telemetry_hz = round(1.0 / avg, 2) if avg > 0 else 0.0
            prev = (self.state.x, self.state.y, self._last_rx)
            self._last_rx = now
            for k, v in fields.items():
                setattr(self.state, k, v)
            if "speed" not in fields and prev[2] is not None and now > prev[2]:
                inst = math.hypot(self.state.x - prev[0], self.state.y - prev[1]) / (now - prev[2])
                self.state.speed = round(0.6 * self.state.speed + 0.4 * inst, 3)
            self.state.last_update = now
            self.state.online = True
            self.rx_count += 1

    def snapshot(self):
        """Current normalized state, with staleness applied."""
        with self.lock:
            s = self.state
            if s.online and (time.time() - s.last_update > self.stale_after):
                s.online = False
            if not s.online:
                s.state = OFFLINE
            return RobotState(**{k: getattr(s, k) for k in s.__dataclass_fields__})

    def info(self):
        return {"id": self.id, "vendor": self.state.vendor, "model": self.state.model,
                "protocol": self.protocol, "endpoint": self.cfg.get("endpoint", {}),
                "capabilities": sorted(self.capabilities),
                "supports_speed_limit": self.supports_speed_limit,
                "supports_native_actions": self.supports_native_actions,
                "nominal_hz": self.nominal_hz}
