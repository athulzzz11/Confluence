"""Vendor 2 - "Atlas Mobility MT-600": REST/HTTP API, polled by the client.

Interface (what the vendor documents):
  GET    /api/v1/status   -> {"position":{"x_mm","y_mm","orientation_deg"},
                              "battery_percentage", "state_text": Ready|Executing|
                              Pause|EmergencyStop|Charging|Error,
                              "velocity_mm_s", "errors":[...], "goal": {...}|null}
  POST   /api/v1/move     {"x_mm", "y_mm", "max_speed_mm_s"?}   replace current goal
  DELETE /api/v1/move     cancel
  PUT    /api/v1/state    {"state": "Pause" | "Ready"}
No push telemetry - clients must poll.
"""
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATE_TEXT = {"driving": "Executing", "idle": "Ready", "paused": "Pause", "obstacle": "EmergencyStop",
              "charging": "Charging", "error": "Error", "action": "Executing"}


class RestApi:
    def __init__(self, fw, host, port):
        self.fw = fw
        api = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body):
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _body(self):
                n = int(self.headers.get("Content-Length") or 0)
                return json.loads(self.rfile.read(n) or b"{}")

            def do_GET(self):
                if self.path != "/api/v1/status":
                    return self._send(404, {"error": "not found"})
                self._send(200, api.status())

            def do_POST(self):
                if self.path != "/api/v1/move":
                    return self._send(404, {"error": "not found"})
                b = self._body()
                try:
                    goal = {"x": float(b["x_mm"]) / 1000.0, "y": float(b["y_mm"]) / 1000.0}
                except (KeyError, ValueError):
                    return self._send(400, {"error": "x_mm and y_mm required"})
                if b.get("max_speed_mm_s"):
                    goal["speed"] = float(b["max_speed_mm_s"]) / 1000.0
                api.fw.set_goals([goal])
                self._send(202, {"accepted": True})

            def do_DELETE(self):
                api.fw.cancel()
                self._send(200, {"cancelled": True})

            def do_PUT(self):
                if self.path != "/api/v1/state":
                    return self._send(404, {"error": "not found"})
                s = self._body().get("state")
                if s == "Pause":
                    api.fw.pause()
                elif s == "Ready":
                    api.fw.resume()
                else:
                    return self._send(400, {"error": "state must be Pause or Ready"})
                self._send(200, {"state": s})

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def status(self):
        fw = self.fw
        x, y, yaw = fw.pose()
        g = fw.goals[0] if fw.goals else None
        return {
            "position": {"x_mm": round(x * 1000), "y_mm": round(y * 1000),
                         "orientation_deg": round(math.degrees(yaw), 1)},
            "battery_percentage": round(fw.battery, 1),
            "state_text": STATE_TEXT[fw.mode()],
            "velocity_mm_s": round(fw.measured_speed * 1000),
            "errors": [{"code": e} for e in fw.errors],
            "goal": {"x_mm": round(g["x"] * 1000), "y_mm": round(g["y"] * 1000)} if g else None,
        }

    def tick(self):
        pass
