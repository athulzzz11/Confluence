"""Legacy TCP line-protocol robots (e.g. Tugwell TX-2).
cm, compass heading (0 = north, clockwise), battery in volts, no speed limits."""
import math
import socket
import threading

from confluence.adapters.base import RobotAdapter
from confluence.model import CHARGING, ERROR, IDLE, MOVING, PAUSED

STATUS_MAP = {"RDY": IDLE, "RUN": MOVING, "HLD": PAUSED, "OBS": MOVING, "CHG": CHARGING, "ERR": ERROR}


class TcpAdapter(RobotAdapter):
    protocol = "tcp"
    supports_speed_limit = False
    nominal_hz = 2.0

    def __init__(self, cfg):
        super().__init__(cfg)
        ep = cfg["endpoint"]
        self.host, self.port = ep["host"], int(ep["port"])
        self.v_empty, self.v_full = ep.get("battery_v_empty", 22.0), ep.get("battery_v_full", 25.2)
        self.sock = None
        self.wlock = threading.Lock()
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=3)
                s.settimeout(5)
                self.sock = s
                for line in s.makefile("r"):
                    if line.startswith("$POS"):
                        self._parse(line.strip())
            except OSError:
                pass
            self.sock = None
            self._stop.wait(1.0)          # reconnect

    def _parse(self, line):
        try:
            _, xcm, ycm, compass, volts, status = line.split(",")
            frac = (float(volts) - self.v_empty) / (self.v_full - self.v_empty)
            yaw = math.radians(90.0 - float(compass))                    # compass -> ROS yaw
            self._received(x=float(xcm) / 100.0, y=float(ycm) / 100.0,
                           yaw=math.atan2(math.sin(yaw), math.cos(yaw)),
                           battery=round(max(0.0, min(1.0, frac)) * 100.0, 1),
                           state=STATUS_MAP.get(status, ERROR), charging=status == "CHG",
                           safety_stop=status == "OBS", errors=["ERR"] if status == "ERR" else [],
                           raw={"telegram": line})
        except ValueError:
            pass

    def _send(self, line):
        if not self.link_up() or self.sock is None:
            return False
        try:
            with self.wlock:
                self.sock.sendall((line + "\n").encode())
            self.tx_count += 1
            return True
        except OSError:
            return False

    def drive(self, start, waypoints, extend=False, action=None, action_seconds=None):
        if not waypoints:
            return False
        w = waypoints[-1]                  # speed limits cannot be expressed in this protocol
        return self._send(f"GOTO {w['x'] * 100:.0f} {w['y'] * 100:.0f}")

    def cancel(self):
        return self._send("ABORT")

    def pause(self):
        return self._send("HOLD")

    def resume(self):
        return self._send("RELEASE")
