"""Vendor 3 - "Tugwell TX-2": legacy TCP line protocol (older tugger fleets).

Interface (what the vendor documents):
  Client -> robot (ASCII lines):
    GOTO <x_cm> <y_cm>      drive to point        -> "ACK GOTO"
    HOLD                    pause                  -> "ACK HOLD"
    RELEASE                 resume                 -> "ACK RELEASE"
    ABORT                   cancel goal            -> "ACK ABORT"
  Robot -> client, pushed at 2 Hz:
    $POS,<x_cm>,<y_cm>,<compass_deg>,<battery_V>,<status>
      compass_deg: 0 = north, clockwise (NOT the ROS convention)
      battery_V:   22.0 V (empty) .. 25.2 V (full)
      status:      RDY | RUN | HLD | OBS | CHG | ERR
  No speed-limit command exists in this protocol.
"""
import math
import socketserver
import threading
import time

STATUS = {"driving": "RUN", "idle": "RDY", "paused": "HLD", "obstacle": "OBS",
          "charging": "CHG", "error": "ERR", "action": "RUN"}
V_EMPTY, V_FULL = 22.0, 25.2


class TcpApi:
    push_hz = 2.0

    def __init__(self, fw, host, port):
        self.fw = fw
        api = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                alive = {"ok": True}
                wlock = threading.Lock()

                def send(line):
                    with wlock:
                        self.wfile.write((line + "\n").encode())

                def pusher():
                    while alive["ok"]:
                        try:
                            send(api.telegram())
                        except OSError:
                            alive["ok"] = False
                            break
                        time.sleep(1.0 / api.push_hz)

                threading.Thread(target=pusher, daemon=True).start()
                try:
                    for raw in self.rfile:
                        reply = api.handle(raw.decode(errors="ignore").strip())
                        send(reply)
                except OSError:
                    pass
                alive["ok"] = False

        socketserver.ThreadingTCPServer.allow_reuse_address = True
        self.server = socketserver.ThreadingTCPServer((host, port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def telegram(self):
        fw = self.fw
        x, y, yaw = fw.pose()
        compass = (90.0 - math.degrees(yaw)) % 360.0
        volts = V_EMPTY + (V_FULL - V_EMPTY) * fw.battery / 100.0
        return f"$POS,{x * 100:.0f},{y * 100:.0f},{compass:.1f},{volts:.2f},{STATUS[fw.mode()]}"

    def handle(self, line):
        parts = line.split()
        if not parts:
            return "NAK EMPTY"
        cmd = parts[0].upper()
        if cmd == "GOTO" and len(parts) == 3:
            try:
                self.fw.set_goals([{"x": float(parts[1]) / 100.0, "y": float(parts[2]) / 100.0}])
            except ValueError:
                return "NAK GOTO"
            return "ACK GOTO"
        if cmd == "HOLD":
            self.fw.pause()
        elif cmd == "RELEASE":
            self.fw.resume()
        elif cmd == "ABORT":
            self.fw.cancel()
        else:
            return f"NAK {cmd}"
        return f"ACK {cmd}"

    def tick(self):
        pass
