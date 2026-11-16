"""Vendor 1 - "Nimbus Robotics AMR-S": a ROS 2-native robot.

Interface (what the vendor documents):
  sub  <ns>/api/goal_pose    geometry_msgs/PoseStamped   drive to pose (map frame)
  sub  <ns>/api/command      std_msgs/String             "pause" | "resume" | "cancel"
                                                         | "escort on" | "escort off" (guide mode:
                                                         the robot waits for the person following it)
  sub  <ns>/api/speed_limit  std_msgs/Float32            m/s, 0 = no limit
  pub  <ns>/api/status       std_msgs/String (JSON)      10 Hz
        {"pose":{"x","y","yaw"}, "battery": 0..1, "mode": NAVIGATING|IDLE|PAUSED|
         BLOCKED|DOCKED_CHARGING|FAULT, "linear_velocity": m/s, "faults": [...]}
"""
import json

MODES = {"driving": "NAVIGATING", "idle": "IDLE", "paused": "PAUSED", "obstacle": "BLOCKED",
         "charging": "DOCKED_CHARGING", "error": "FAULT", "action": "NAVIGATING"}


class RosNativeApi:
    rate_hz = 10.0

    def __init__(self, fw, bus, ns):
        self.fw, self.bus, self.ns = fw, bus, ns
        self.speed_limit = 0.0
        bus.subscribe(f"{ns}/api/goal_pose", self._on_goal)
        bus.subscribe(f"{ns}/api/command", self._on_cmd)
        bus.subscribe(f"{ns}/api/speed_limit", self._on_limit)

    def _on_goal(self, pose):
        x, y, _ = pose
        self.fw.set_goals([{"x": x, "y": y, "speed": self.speed_limit or None}])

    def _on_cmd(self, cmd):
        cmd = cmd.strip().lower()
        if cmd == "pause":
            self.fw.pause()
        elif cmd == "resume":
            self.fw.resume()
        elif cmd == "cancel":
            self.fw.cancel()
        elif cmd in ("escort on", "escort off"):
            self.fw.escort = cmd.endswith("on")
            if not self.fw.escort:
                self.fw.guest_distance = None

    def _on_limit(self, v):
        self.speed_limit = max(0.0, v)
        with self.fw.lock:
            for g in self.fw.goals:
                g["speed"] = self.speed_limit or None

    def tick(self):
        x, y, yaw = self.fw.pose()
        msg = {
            "pose": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)},
            "battery": round(self.fw.battery / 100.0, 3),
            "mode": MODES[self.fw.mode()],
            "linear_velocity": round(self.fw.measured_speed, 3),
            "faults": list(self.fw.errors),
            "escort": {"active": self.fw.escort, "waiting_for_guest": self.fw.waiting_for_guest,
                       "guest_distance": (round(self.fw.guest_distance, 2)
                                          if self.fw.guest_distance is not None else None)},
        }
        self.bus.publish(f"{self.ns}/api/status", json.dumps(msg))
