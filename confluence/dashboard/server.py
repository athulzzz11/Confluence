"""Dashboard web server (stdlib only). Serves the UI and a small JSON API that
any front end (this page, a React app, scripts) can use.

GET  /api/state                 fleet snapshot + recent events
GET  /api/site                  site map, stations, zones, task templates
GET  /api/report                audit report (HTML)
POST /api/task                  {"kind", "pickup"?, "dropoff"?}
POST /api/robot/<id>/pause|resume          instant stop / continue (robot's own pause)
POST /api/robot/<id>/goto      {"target": "gate_2" | "CN4"} or {"x": 3.0, "y": -1.4}
POST /api/robot/<id>/jog       {"direction": "north|south|east|west"}
POST /api/robot/<id>/stop      stop at the next reserved node
POST /api/robot/<id>/home      back to its charger, rejoins automatic work
POST /api/robot/<id>/mode      {"mode": "auto" | "manual"}
POST /api/robot/<id>/linkloss  {"seconds": 10}
POST /api/task/<id>/assign     {"robot": "r2"}
POST /api/task/<id>/cancel
POST /api/estop | /api/release
POST /api/auto                  {"on": true|false}
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from confluence import site

HERE = os.path.dirname(__file__)


class Dashboard:
    def __init__(self, orch, compliance, port=8080, host="0.0.0.0"):
        self.orch, self.compliance, self.port, self.host = orch, compliance, port, host

    def start(self):
        dash = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype="application/json"):
                data = body if isinstance(body, bytes) else (
                    body.encode() if isinstance(body, str) else json.dumps(body, default=str).encode())
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(data)

            def _json(self):
                n = int(self.headers.get("Content-Length") or 0)
                try:
                    return json.loads(self.rfile.read(n) or b"{}")
                except ValueError:
                    return {}

            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_GET(self):
                p = self.path.split("?")[0]
                if p in ("/", "/index.html"):
                    with open(os.path.join(HERE, "index.html"), "rb") as f:
                        return self._send(200, f.read(), "text/html; charset=utf-8")
                if p == "/api/state":
                    snap = dash.orch.snapshot()
                    snap["events"] = dash.compliance.recent_events(80)
                    return self._send(200, snap)
                if p == "/api/site":
                    d = site.as_dict()
                    d["task_templates"] = {k: {"capability": c, "pickups": a, "dropoffs": b}
                                           for k, (c, a, b) in site.TASK_TEMPLATES.items()}
                    return self._send(200, d)
                if p == "/api/report":
                    return self._send(200, dash.compliance.report(dash.orch.snapshot()), "text/html; charset=utf-8")
                self._send(404, {"error": "not found"})

            def do_POST(self):
                p, b, o = self.path.split("?")[0], self._json(), dash.orch
                try:
                    if p == "/api/task":
                        t = o.add_task(b["kind"], b.get("pickup") or None, b.get("dropoff") or None, source="operator")
                        return self._send(200, t.to_dict())
                    if p == "/api/estop":
                        o.estop()
                        return self._send(200, {"ok": True})
                    if p == "/api/release":
                        o.release_estop()
                        return self._send(200, {"ok": True})
                    if p == "/api/auto":
                        o.set_auto(b.get("on", True))
                        return self._send(200, {"ok": True})
                    parts = p.strip("/").split("/")
                    if len(parts) == 4 and parts[:2] == ["api", "robot"] and parts[2] in o.robots:
                        rid, act = parts[2], parts[3]
                        if act == "pause":
                            return self._send(200, {"ok": o.pause(rid)})
                        if act == "resume":
                            return self._send(200, {"ok": o.resume(rid)})
                        if act == "linkloss":
                            o.link_loss(rid, float(b.get("seconds", 10)))
                            return self._send(200, {"ok": True})
                        if act == "goto":
                            target = (b["x"], b["y"]) if "x" in b and "y" in b else b["target"]
                            return self._send(200, {"ok": True, "node": o.goto(rid, target)})
                        if act == "jog":
                            return self._send(200, {"ok": True, "node": o.jog(rid, b.get("direction", ""))})
                        if act == "stop":
                            return self._send(200, {"ok": True, "node": o.stop_robot(rid)})
                        if act == "home":
                            return self._send(200, {"ok": True, "node": o.send_home(rid)})
                        if act == "mode":
                            return self._send(200, {"ok": o.set_mode(rid, b.get("mode", ""))})
                    if len(parts) == 4 and parts[:2] == ["api", "task"]:
                        tid, act = parts[2], parts[3]
                        if act == "assign":
                            return self._send(200, {"ok": o.assign_task(tid, b.get("robot", ""))})
                        if act == "cancel":
                            return self._send(200, {"ok": o.cancel_task(tid)})
                except (KeyError, ValueError) as e:
                    return self._send(400, {"error": str(e)})
                self._send(404, {"error": "not found"})

        self.server = ThreadingHTTPServer((self.host, self.port), H)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
