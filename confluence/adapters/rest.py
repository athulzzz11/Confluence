"""REST/HTTP robots (e.g. Atlas MT-600): poll /status, POST /move. mm + degrees."""
import json
import math
import threading
import time
import urllib.error
import urllib.request

from confluence.adapters.base import RobotAdapter
from confluence.model import CHARGING, ERROR, IDLE, MOVING, PAUSED

STATE_MAP = {"Ready": IDLE, "Executing": MOVING, "Pause": PAUSED, "EmergencyStop": MOVING,
             "Charging": CHARGING, "Error": ERROR}


class RestAdapter(RobotAdapter):
    protocol = "rest"
    supports_speed_limit = True
    nominal_hz = 2.0

    def __init__(self, cfg):
        super().__init__(cfg)
        self.url = cfg["endpoint"]["url"].rstrip("/")
        self.poll_hz = cfg["endpoint"].get("poll_hz", 2.0)
        self.nominal_hz = self.poll_hz
        self._stop = threading.Event()

    def _req(self, method, path, body=None, timeout=1.0):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.url + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")

    def start(self):
        threading.Thread(target=self._poll, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _poll(self):
        while not self._stop.is_set():
            t0 = time.time()
            if self.link_up():
                try:
                    m = self._req("GET", "/api/v1/status")
                    p = m["position"]
                    st = m["state_text"]
                    self._received(x=p["x_mm"] / 1000.0, y=p["y_mm"] / 1000.0,
                                   yaw=math.radians(p["orientation_deg"]),
                                   battery=float(m["battery_percentage"]),
                                   speed=m.get("velocity_mm_s", 0) / 1000.0,
                                   state=STATE_MAP.get(st, ERROR), charging=st == "Charging",
                                   safety_stop=st == "EmergencyStop",
                                   errors=[e.get("code", str(e)) for e in m.get("errors", [])], raw=m)
                except (urllib.error.URLError, OSError, ValueError, KeyError):
                    pass   # staleness check marks the robot OFFLINE
            self._stop.wait(max(0.0, 1.0 / self.poll_hz - (time.time() - t0)))

    def _send(self, method, path, body=None):
        if not self.link_up():
            return False
        try:
            self._req(method, path, body)
            self.tx_count += 1
            return True
        except (urllib.error.URLError, OSError, ValueError):
            return False

    def drive(self, start, waypoints, extend=False, action=None, action_seconds=None):
        if not waypoints:
            return False
        w = waypoints[-1]
        body = {"x_mm": round(w["x"] * 1000), "y_mm": round(w["y"] * 1000)}
        if w.get("speed"):
            body["max_speed_mm_s"] = round(w["speed"] * 1000)
        return self._send("POST", "/api/v1/move", body)

    def cancel(self):
        return self._send("DELETE", "/api/v1/move")

    def pause(self):
        return self._send("PUT", "/api/v1/state", {"state": "Pause"})

    def resume(self):
        return self._send("PUT", "/api/v1/state", {"state": "Ready"})
