"""Walking passengers for the airport terminal.

They are deliberately NOT robots and NOT known to Confluence - just people
crossing the concourse. Robots react to them only through their own onboard
safety (lidar protective field), which is exactly the real-world situation.
People walk back and forth along their route, pause at the ends, and wait
politely when a robot is right in front of them.
"""
import math
import random


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Walker:
    SPEED = 0.7
    COURTESY_DIST = 1.1                 # stop if a robot is this close ahead
    COURTESY_ANGLE = math.radians(50)

    def __init__(self, name, route, body, robot_bodies, seed=0):
        self.name, self.route, self.body = name, list(route), body
        self.robots = robot_bodies
        self.idx = 1
        self.direction = 1
        self.rng = random.Random(seed)
        self.SPEED = self.rng.uniform(0.55, 0.9)      # people walk at different paces
        self.pause = self.rng.uniform(0.5, 4.0)
        self.waited = 0.0

    def _robot_ahead(self, x, y, heading):
        for rb in self.robots:
            rx, ry, _ = rb.pose()
            d = math.hypot(rx - x, ry - y)
            if d < self.COURTESY_DIST and abs(_wrap(math.atan2(ry - y, rx - x) - heading)) < self.COURTESY_ANGLE:
                return True
        return False

    def step(self, dt):
        x, y, yaw = self.body.pose()
        if self.pause > 0:
            self.pause -= dt
            self.body.set_cmd(0.0, 0.0)
            return
        gx, gy = self.route[self.idx]
        d = math.hypot(gx - x, gy - y)
        if d < 0.12:
            nxt = self.idx + self.direction
            if not 0 <= nxt < len(self.route):
                self.direction *= -1
                nxt = self.idx + self.direction
            self.idx = nxt
            self.pause = self.rng.uniform(2.0, 7.0)          # stop, look at the boards, carry on
            self.body.set_cmd(0.0, 0.0)
            return
        heading = math.atan2(gy - y, gx - x)
        err = _wrap(heading - yaw)
        w = max(-2.0, min(2.0, 3.0 * err))
        v = 0.0 if abs(err) > 0.4 else min(self.SPEED, d + 0.1) * math.cos(err)
        if v > 0 and self._robot_ahead(x, y, yaw):
            self.waited += dt
            v = 0.0
            if self.waited > 8.0:                               # give up and turn back
                self.direction *= -1
                self.idx = max(0, min(len(self.route) - 1, self.idx + self.direction))
                self.waited = 0.0
        else:
            self.waited = 0.0
        self.body.set_cmd(v, w)


class Guest:
    """A passenger who gets guided: waits by the info desk, walks behind the guide
    robot while it is in escort mode, stays at the destination for a while, then
    walks back to the desk (standing in for the next passenger).

    The link to the robot is the robot's own escort mode (set through its vendor
    API): the person follows while it is on, and the robot's rear sensor measures
    how far behind they are (`firmware.guest_distance`)."""
    FOLLOW_GAP = 1.15      # m behind the robot
    SPEED = 0.95

    def __init__(self, name, cfg, body, firmware, robot_body, other_robot_bodies, seed=0):
        self.name, self.cfg, self.body = name, cfg, body
        self.fw, self.robot = firmware, robot_body
        self.others = other_robot_bodies
        self.state = "WAIT"
        self.path = []
        self.stay_until = 0.0
        self.rng = random.Random(seed)
        self.t = 0.0
        self.blocked = 0.0

    def _go(self, x, y, tx, ty, yaw, speed, dt, avoid=None):
        """Steer toward (tx, ty); `avoid` = (x, y, radius) to walk around."""
        vx, vy = tx - x, ty - y
        d = math.hypot(vx, vy)
        if avoid:
            ax, ay, rad = avoid
            ox, oy = x - ax, y - ay
            od = math.hypot(ox, oy) or 1e-6
            if od < rad:                                  # too close: walk round it, never through it
                tx_, ty_ = -oy / od, ox / od              # tangent; pick the side that leads to the target
                if tx_ * vx + ty_ * vy < 0:
                    tx_, ty_ = -tx_, -ty_
                out = 1.6 * (rad - od) / rad + (0.9 if od < 0.75 else 0.0)
                keep = 0.0 if od < 0.75 else 0.5          # inside 0.75 m ignore the target entirely
                vx = keep * vx / (d or 1) + ox / od * out + tx_ * 0.9
                vy = keep * vy / (d or 1) + oy / od * out + ty_ * 0.9
        if d < 0.1:
            self.body.set_cmd(0.0, 0.0)
            return True
        err = _wrap(math.atan2(vy, vx) - yaw)
        w = max(-2.5, min(2.5, 3.5 * err))
        v = 0.0 if abs(err) > 0.6 else min(speed, 1.2 * d + 0.1) * max(0.0, math.cos(err))
        self.body.set_cmd(v, w)
        return False

    def _robot_in_the_way(self, x, y, yaw):
        for rb in self.others + [self.robot]:
            rx, ry, _ = rb.pose()
            if math.hypot(rx - x, ry - y) < 1.0 and abs(_wrap(math.atan2(ry - y, rx - x) - yaw)) < math.radians(50):
                return True
        return False

    def step(self, dt):
        self.t += dt
        x, y, yaw = self.body.pose()
        rx, ry, ryaw = self.robot.pose()
        dist = math.hypot(rx - x, ry - y)
        escort = self.fw.escort

        if escort:
            self.fw.guest_distance = dist                  # the robot waits until its passenger is near
            if self.state in ("WAIT", "FOLLOW"):
                self.state = "FOLLOW"
            elif self.state == "STAY":                     # next guided trip is starting: head back now
                self.stay_until = 0.0
        if self.state == "FOLLOW":
            if not escort:                                 # arrived: step aside and stay a while
                _ox, oy = self.cfg["stay_offset"]
                self.path = [(x, ry + oy)]                 # step sideways off the robot lane
                self.state = "LEAVE"
                self.fw.guest_distance = None
            else:
                self.fw.guest_distance = dist              # what the robot's rear sensor sees
                tx = rx - self.FOLLOW_GAP * math.cos(ryaw)
                ty = ry - self.FOLLOW_GAP * math.sin(ryaw)
                self._go(x, y, tx, ty, yaw, self.SPEED, dt, avoid=(rx, ry, 1.0))
                return
        if self.state == "WAIT":
            self.fw.guest_distance = None
            wx, wy = self.cfg["wait"]
            if math.hypot(wx - x, wy - y) > 0.25:
                self._go(x, y, wx, wy, yaw, 0.6, dt)
            else:
                self.body.set_cmd(0.0, 0.0)
            return
        if self.state == "STAY":
            self.body.set_cmd(0.0, 0.0)
            if self.t >= self.stay_until:
                self.path = list(self.cfg["return_via"]) + [self.cfg["wait"]]
                self.path[0] = (self.path[0][0], y) if abs(self.path[0][1] - y) < 0.6 else self.path[0]
                self.state = "RETURN"
            return
        # LEAVE / RETURN: walk the remaining path, giving way to robots
        if not self.path:
            if self.state == "LEAVE":
                self.state, self.stay_until = "STAY", self.t + self.cfg["stay_s"] * self.rng.uniform(0.8, 1.3)
            else:
                self.state = "WAIT"
            return
        if self.state == "RETURN" and self._robot_in_the_way(x, y, yaw) and self.blocked < 6.0:
            self.blocked += dt                             # let the robot pass (but not forever)
            self.body.set_cmd(0.0, 0.0)
            return
        if not self._robot_in_the_way(x, y, yaw):
            self.blocked = 0.0
        hurry = 1.25 if (escort and self.state == "RETURN") else 0.75   # next robot is already waiting
        near = min(self.others + [self.robot], key=lambda b: math.hypot(b.pose()[0] - x, b.pose()[1] - y))
        nx, ny, _ = near.pose()
        if self._go(x, y, self.path[0][0], self.path[0][1], yaw, hurry, dt, avoid=(nx, ny, 0.95)):
            self.path.pop(0)
