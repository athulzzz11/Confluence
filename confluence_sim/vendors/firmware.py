"""Simulated onboard robot software ("vendor firmware").

This stands in for the software a robot ships with from its manufacturer:
local navigation to a goal, its own safety stop, battery management and
docking. Confluence never imports this module - it only talks to the robot
through the vendor API wrapped around it (VDA 5050, REST, TCP, ROS 2).

A Firmware drives a Body:
  * KinematicBody - pure-Python physics, for running without Gazebo
  * GazeboBody    - cmd_vel / odom / lidar through ROS 2 (see ros_io.py)
"""
import math
import threading
import time

from confluence import site

SAFETY_STOP_DIST = 0.45     # m, robot's own protective field (front)
SAFETY_HALF_ANGLE = math.radians(25)
ARRIVE_TOL = 0.08
GUEST_MAX_GAP = 2.4         # m, a guide robot stops and waits when its passenger is further behind
GUEST_WAIT_TIMEOUT = 30.0   # s, then it carries on (passenger gave up)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ------------------------------------------------------------------ bodies
class KinematicWorld:
    """Shared world for kinematic bodies so they can 'see' each other."""

    def __init__(self):
        self.bodies = []
        self.lock = threading.Lock()

    def add(self, body):
        self.bodies.append(body)
        body.world = self


class KinematicBody:
    def __init__(self, name, x, y, yaw, length, width):
        self.name, self.x, self.y, self.yaw = name, x, y, yaw
        self.half_len, self.half_w = length / 2, width / 2
        self.v = self.w = 0.0
        self.world = None
        self.odometer = 0.0

    def pose(self):
        return self.x, self.y, self.yaw

    def set_cmd(self, v, w):
        self.v, self.w = v, w

    def step(self, dt):
        self.yaw = wrap(self.yaw + self.w * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.odometer += abs(self.v) * dt

    def obstacle_ahead(self, reverse=False):
        """Gap (m) to the nearest robot inside the protective field."""
        heading = self.yaw + (math.pi if reverse else 0.0)
        best = None
        for o in self.world.bodies:
            if o is self:
                continue
            dx, dy = o.x - self.x, o.y - self.y
            if abs(wrap(math.atan2(dy, dx) - heading)) > math.radians(40):
                continue
            gap = math.hypot(dx, dy) - self.half_len - max(o.half_len, o.half_w) * 0.8
            best = gap if best is None else min(best, gap)
        return best


# ------------------------------------------------------------------ firmware
class Firmware:
    """Goal-following controller with the behaviour real AMRs have:
    turn-then-drive, reversing out of docks, protective-field stop,
    pause/resume, battery drain and charging when docked."""

    def __init__(self, name, body, max_speed, accepts_speed_limit=True):
        self.name = name
        self.body = body
        self.max_speed = max_speed
        self.accepts_speed_limit = accepts_speed_limit
        self.lock = threading.RLock()
        self.goals = []              # list of dicts {x, y, node_id, speed, actions}
        self.paused = False
        self.safety_stop = False
        # escort mode (guide robots): the robot tracks the person walking behind it
        self.escort = False
        self.guest_distance = None    # metres to the person it is guiding (rear sensor), None = not seen
        self.waiting_for_guest = False
        self._guest_wait_since = None
        self.errors = []
        self.battery = 85.0
        self.charging = False
        self.action = None           # {"type", "id", "until"}
        self.finished_actions = []   # [(action_id, type, "FINISHED")]
        self.last_node = None
        self.last_seq = 0
        self.v_cmd = 0.0
        self.measured_speed = 0.0
        self._last_pose = None
        self._last_t = None

    # ---------------- commands (called by vendor API layers)
    def set_goals(self, goals):
        with self.lock:
            self.goals = list(goals)
            self.errors = [e for e in self.errors if e != "GOAL_UNREACHABLE"]

    def append_goals(self, goals):
        with self.lock:
            self.goals.extend(goals)

    def cancel(self):
        with self.lock:
            self.goals = []
            self.action = None

    def pause(self):
        with self.lock:
            self.paused = True

    def resume(self):
        with self.lock:
            self.paused = False

    def start_action(self, action_type, action_id, duration):
        with self.lock:
            self.action = {"type": action_type, "id": action_id, "until": time.time() + duration}

    # ---------------- status
    @property
    def busy(self):
        return bool(self.goals) or self.action is not None

    def mode(self):
        if self.errors:
            return "error"
        if self.paused:
            return "paused"
        if self.action:
            return "action"
        if self.safety_stop:
            return "obstacle"
        if self.goals:
            return "driving"
        if self.charging:
            return "charging"
        return "idle"

    def pose(self):
        return self.body.pose()

    # ---------------- control loop
    def _docking(self, x, y):
        return any(math.hypot(x - cx, y - cy) < 0.75 for cx, cy in site.CHARGERS.values())

    def step(self, dt):
        with self.lock:
            x, y, yaw = self.body.pose()
            now = time.time()
            if self._last_pose is not None and now > self._last_t:
                d = math.hypot(x - self._last_pose[0], y - self._last_pose[1])
                inst = d / (now - self._last_t)
                self.measured_speed = 0.8 * self.measured_speed + 0.2 * inst
                self.battery = max(0.0, self.battery - 0.35 * d)
            self._last_pose, self._last_t = (x, y), now
            self.battery = max(0.0, self.battery - 0.004 * dt)

            # native action (pick/drop) in progress
            if self.action:
                self.body.set_cmd(0.0, 0.0)
                if now >= self.action["until"]:
                    self.finished_actions.append((self.action["id"], self.action["type"], "FINISHED"))
                    self.action = None
                return

            docked = any(math.hypot(x - cx, y - cy) < 0.15 for cx, cy in site.CHARGERS.values())
            self.charging = docked and not self.goals
            if self.charging:
                self.battery = min(100.0, self.battery + 1.5 * dt)

            if self.paused or not self.goals:
                self.safety_stop = False
                self.body.set_cmd(0.0, 0.0)
                self.v_cmd = 0.0
                return

            g = self.goals[0]
            dx, dy = g["x"] - x, g["y"] - y
            d = math.hypot(dx, dy)
            if d < ARRIVE_TOL:
                self.last_node = g.get("node_id")
                self.last_seq = g.get("seq", self.last_seq)
                pending = g.get("actions") or []
                self.goals.pop(0)
                self.body.set_cmd(0.0, 0.0)
                if pending:
                    a = pending[0]
                    self.start_action(a["type"], a["id"], a.get("duration", 4.0))
                return

            # escort mode: don't run away from the passenger - wait if they fall behind
            self.waiting_for_guest = False
            if self.escort and self.guest_distance is not None and self.guest_distance > GUEST_MAX_GAP:
                self._guest_wait_since = self._guest_wait_since or now
                if now - self._guest_wait_since < GUEST_WAIT_TIMEOUT:
                    self.waiting_for_guest = True
                    self.safety_stop = False
                    self.v_cmd = 0.0
                    self.body.set_cmd(0.0, 0.0)
                    return
            else:
                self._guest_wait_since = None

            heading = math.atan2(dy, dx)
            err = wrap(heading - yaw)
            reverse = d < 1.2 and abs(err) > math.radians(120)   # back out of docks/spurs
            if reverse:
                err = wrap(err + math.pi)

            vmax = self.max_speed
            if self.accepts_speed_limit and g.get("speed"):
                vmax = min(vmax, g["speed"])

            w = max(-1.2, min(1.2, 2.0 * err))
            if abs(err) > 0.3:
                v = 0.0                                   # turn on the spot first
            else:
                v = min(vmax, 0.9 * d + 0.06) * math.cos(err)
            if reverse:
                v = -min(abs(v), 0.15)

            # robot's own protective field - Confluence cannot override this
            gap = self.body.obstacle_ahead(reverse=reverse)
            muted = self._docking(x, y) or self._docking(g["x"], g["y"]) and d < 0.75
            if v != 0.0 and gap is not None and gap < SAFETY_STOP_DIST and not muted:
                self.safety_stop = True
                v = 0.0
                w = 0.0
            else:
                self.safety_stop = False
            # smooth acceleration
            self.v_cmd += max(-0.6 * dt, min(0.6 * dt, v - self.v_cmd)) if v != 0 else -self.v_cmd
            self.body.set_cmd(self.v_cmd, w)
