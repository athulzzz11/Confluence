"""Orchestration engine: task allocation + traffic control across vendors.

Traffic control works like railway block signalling on the site roadmap:
a robot may only drive into roadmap nodes it has *reserved*, and a node is
reserved by at most one robot. Robots are released a few nodes ahead at a
time (lookahead along straight segments) and give nodes back once they have
cleared them. This keeps robots apart regardless of how good each vendor's
own navigation is, and works through the lowest common denominator of every
protocol ("go to this point").

When robots wait on each other in a cycle (e.g. head-on in a lane), the
lowest-priority robot in the cycle is re-routed around the others.
"""
import math
import random
import threading
import time

from confluence import site
from confluence.model import ERROR, OFFLINE, Task

LOOKAHEAD = 3            # max nodes reserved ahead of the robot
PASS_RADIUS = 0.35       # robot "has reached" an intermediate node
STOP_RADIUS = 0.22       # robot has arrived at its final node
RELEASE_DIST = 1.25      # give a node back once the robot is this far from it (keeps bodies apart)
DWELL_S = 4.0            # emulated pick/drop time for robots without native actions
MIN_BATTERY = 30.0       # don't assign work below this
RESEND_S = 1.5


class RobotCtx:
    def __init__(self, adapter, cfg):
        self.a = adapter
        self.id = adapter.id
        self.priority = int(cfg.get("priority", 1))
        self.home = cfg.get("home_charger")
        self.current = None          # last node the robot reached
        self.path = []               # remaining planned nodes (after current)
        self.ahead = []              # reserved nodes ahead (prefix of path)
        self.behind = []             # reserved nodes the robot is still clearing
        self.sent_until = None       # last node included in a drive() command
        self.last_send = 0.0
        self.goal_kind = None        # "task" | "home" | "manual"
        self.mode = cfg.get("mode", "auto")   # "auto": gets tasks | "manual": operator drives it
        self.manual_goal = None      # node the operator sent it to
        self.last_jog = None         # direction of the last arrow press (for chaining)
        self.home_first = False      # operator sent it home: no tasks until it's docked
        self.work_label = None       # what it is physically doing right now ("loading the bag")
        self.dwell_started = None
        self.dwell_total = 0.0
        self.rest_until = None       # short pause after finishing a job
        self.escorting = False
        self.task = None
        self.dwell_until = None
        self.native_action = False
        self.waiting_for = None
        self.wait_since = None
        self.operator_paused = False
        self.was_online = False
        self.was_safety = False
        self.odometer = 0.0
        self._last_xy = None
        self.busy_time = 0.0
        self.resume_goal = None      # set while pulled aside to let another robot pass
        self.last_resolve = 0.0

    @property
    def held(self):
        return ([self.current] if self.current else []) + self.ahead + self.behind


class Orchestrator:
    def __init__(self, adapters, robot_cfgs, compliance, auto_tasks=True, task_interval=10.0, seed=None):
        self.robots = {a.id: RobotCtx(a, robot_cfgs[a.id]) for a in adapters}
        self.log = compliance
        self.reserved = {}           # node -> robot id
        self.tasks = []
        self.auto_tasks = auto_tasks
        self.task_interval = task_interval
        self.next_auto = time.time() + 3.0
        self.estopped = False
        self.lock = threading.RLock()
        self.rng = random.Random(seed)
        self.started = time.time()
        self._stop = threading.Event()

    # ================================================================ public API
    def start(self, hz=10.0):
        for r in self.robots.values():
            r.a.start()
        self.log.event("system_start", robots=list(self.robots))

        def loop():
            while not self._stop.is_set():
                t0 = time.time()
                try:
                    self.tick()
                except Exception as e:     # keep the control loop alive, but record it
                    self.log.event("orchestrator_error", error=repr(e))
                self._stop.wait(max(0.0, 1.0 / hz - (time.time() - t0)))

        threading.Thread(target=loop, daemon=True).start()

    def stop(self):
        self._stop.set()
        for r in self.robots.values():
            r.a.stop()

    def add_task(self, kind, pickup=None, dropoff=None, source="operator", priority=1):
        if kind not in site.TASK_TEMPLATES:
            raise ValueError(f"unknown task kind {kind}")
        cap, picks, drops = site.TASK_TEMPLATES[kind]
        pickup = pickup or self.rng.choice(picks)
        dropoff = dropoff or self.rng.choice(drops)
        for s in (pickup, dropoff):
            if s not in site.STATIONS:
                raise ValueError(f"unknown station {s}")
        with self.lock:
            t = Task(kind=kind, capability=cap, pickup=pickup, dropoff=dropoff, source=source, priority=priority)
            self.tasks.append(t)
            self.log.event("task_created", task=t.task_id, kind=kind, capability=cap,
                           pickup=pickup, dropoff=dropoff, source=source)
            return t

    def pause(self, rid, reason="operator"):
        with self.lock:
            r = self.robots[rid]
            r.operator_paused = True
            ok = r.a.pause()
            self.log.event("robot_paused", robot=rid, reason=reason, delivered=ok)
            return ok

    def resume(self, rid, reason="operator"):
        with self.lock:
            r = self.robots[rid]
            if self.estopped and reason == "operator":
                return False
            r.operator_paused = False
            ok = r.a.resume()
            self.log.event("robot_resumed", robot=rid, reason=reason, delivered=ok)
            return ok

    def estop(self):
        with self.lock:
            self.estopped = True
            for rid in self.robots:
                self.pause(rid, reason="fleet_stop")
            self.log.event("fleet_stop")

    def release_estop(self):
        with self.lock:
            self.estopped = False
            for rid in self.robots:
                self.resume(rid, reason="fleet_stop_released")
            self.log.event("fleet_stop_released")

    def link_loss(self, rid, seconds=10.0):
        with self.lock:
            self.robots[rid].a.simulate_link_loss(seconds)
            self.log.event("link_loss_injected", robot=rid, seconds=seconds)

    def set_auto(self, on):
        with self.lock:
            self.auto_tasks = bool(on)
            self.log.event("auto_tasks", enabled=self.auto_tasks)

    # ================================================================ individual robot control
    def _robot(self, rid):
        if rid not in self.robots:
            raise ValueError(f"unknown robot '{rid}'")
        return self.robots[rid]

    def _resolve_target(self, target, rid=None):
        """Station name, node id, or (x, y) -> roadmap node."""
        if isinstance(target, (list, tuple)) and len(target) == 2:
            x, y = float(target[0]), float(target[1])
            own_home = self.robots[rid].home if rid in self.robots else None
            best, bd = None, 1e9
            for nid, (nx, ny) in site.NODES.items():
                if nid in site.CHARGERS and nid != own_home:
                    continue                      # a click never lands in another robot's charger
                d = math.hypot(nx - x, ny - y)
                if d < bd:
                    best, bd = nid, d
            return best
        t = str(target).strip()
        if t.lower() in site.STATIONS:
            return site.STATIONS[t.lower()]
        if t in site.NODES:
            return t
        if t.upper() in site.NODES:
            return t.upper()
        names = ", ".join(sorted(k for k in site.STATIONS if not k.startswith("chg")))
        raise ValueError(f"unknown destination '{target}'. Stations: {names}")

    def _check_target(self, r, node):
        """Refuse destinations that can never be reached, with a clear reason."""
        for o in self.robots.values():
            if o.id == r.id:
                continue
            if node == o.home and node in site.CHARGERS:
                raise ValueError(f"{node} is {o.id}'s charger - pick another spot")
            if node in o.held and self._immobile(o):
                why = "paused" if o.operator_paused else "offline" if o.a.snapshot().state == OFFLINE else "parked"
                raise ValueError(f"{node} is occupied by {o.id} ({why}) - move {o.id} first or pick another spot")

    def _trim_commitment(self, r, keep_if_left=None):
        """Drop reserved-ahead nodes so a new command takes effect right away
        instead of after the robot finishes its old reservation (no U-turns).
        Keeps the node the robot is already driving into once it is past
        `keep_if_left` metres from its current node (default: half way)."""
        if not r.ahead:
            return
        s = r.a.snapshot()
        cx, cy = site.NODES[r.current]
        nx, ny = site.NODES[r.ahead[0]]
        left = math.hypot(s.x - cx, s.y - cy)
        limit = keep_if_left if keep_if_left is not None else 0.5 * math.hypot(nx - cx, ny - cy)
        keep = 1 if left > limit else 0
        for n in r.ahead[keep:]:
            if n not in r.behind:
                r.behind.append(n)       # released once the robot is clear of it
        r.ahead = r.ahead[:keep]
        r.path = list(r.ahead)
        r.sent_until = None              # next drive() is a fresh command, not an extension

    def _release_task(self, r, why):
        """Give the robot's task back to the queue so another robot can take it."""
        t = r.task
        if not t:
            return
        t.status, t.robot_id, t.assigned_at = "QUEUED", None, None
        self._set_escort(r, False)
        r.task = None
        r.dwell_until = r.dwell_started = None
        r.work_label = None
        r.native_action = False
        self.log.event("task_released", task=t.task_id, robot=r.id, reason=why)

    def set_mode(self, rid, mode):
        """auto = Confluence assigns it work; manual = only the operator moves it."""
        if mode not in ("auto", "manual"):
            raise ValueError("mode must be auto or manual")
        with self.lock:
            r = self._robot(rid)
            if r.mode == mode:
                return True
            r.mode = mode
            if mode == "manual":
                self._release_task(r, "switched to manual")
                if r.goal_kind == "home":
                    r.path = list(r.ahead)        # stop at the next reserved node
                    r.goal_kind = "manual" if r.ahead else None
            else:
                r.manual_goal = None
            self.log.event("robot_mode", robot=rid, mode=mode)
            return True

    def goto(self, rid, target, _chain=False):
        """Send one robot to a station / node / point. Puts it in manual mode.
        Still fully traffic-controlled: it only moves into nodes reserved for it."""
        with self.lock:
            r = self._robot(rid)
            if r.current is None:
                raise ValueError(f"{rid} has not reported its position yet")
            node = self._resolve_target(target, rid)
            self._check_target(r, node)
            if r.mode != "manual":
                self.set_mode(rid, "manual")
            self._release_task(r, "operator sent it elsewhere")
            r.home_first = False
            r.rest_until = None
            if not _chain:
                self._trim_commitment(r)
                r.last_jog = None
            if not self._plan(r, node, "manual"):
                raise ValueError(f"no route from {r.current} to {node}")
            r.manual_goal = node
            r.resume_goal = None
            self.log.event("manual_goto", robot=rid, target=str(target), node=node, route=list(r.path))
            return node

    def jog(self, rid, direction):
        """Move one roadmap node north/south/east/west (like arrow keys)."""
        heading = {"north": math.pi / 2, "n": math.pi / 2, "up": math.pi / 2,
                   "south": -math.pi / 2, "s": -math.pi / 2, "down": -math.pi / 2,
                   "east": 0.0, "e": 0.0, "right": 0.0,
                   "west": math.pi, "w": math.pi, "left": math.pi}.get(str(direction).lower())
        if heading is None:
            raise ValueError("direction must be north, south, east or west")
        with self.lock:
            r = self._robot(rid)
            if r.current is None:
                raise ValueError(f"{rid} has not reported its position yet")
            d = str(direction).lower()[0]
            d_full = {"n": "north", "u": "north", "s": "south", "d": "south", "e": "east", "r": "east"}.get(d, "west")
            chaining = r.last_jog == d and r.goal_kind == "manual" and r.path
            if chaining:
                base = r.path[-1]                        # same arrow again: one more node that way
            else:
                self._trim_commitment(r)                 # new direction: start from where it can stop
                base = r.ahead[-1] if r.ahead else r.current
            if base in site.CHARGERS:                    # docked: back out to the lane first
                exit_node = next(iter(site.EDGES[base]))
                (cx, cy), (ex, ey) = site.NODES[base], site.NODES[exit_node]
                out = math.atan2(ey - cy, ex - cx)
                if abs(math.atan2(math.sin(out - heading), math.cos(out - heading))) < math.radians(35):
                    node = self.goto(rid, exit_node, _chain=True)   # arrow points out of the charger
                    r.last_jog = d
                    return node
                base = exit_node                         # leave the charger, then go that way
            bx, by = site.NODES[base]
            best, best_err = None, math.radians(35)
            for m in site.EDGES[base]:
                if m in site.CHARGERS and m != r.home:
                    continue
                mx, my = site.NODES[m]
                err = abs(math.atan2(math.sin(math.atan2(my - by, mx - bx) - heading),
                                     math.cos(math.atan2(my - by, mx - bx) - heading)))
                if err < best_err:
                    best, best_err = m, err
            if best is None:
                # no direct link that way: go to the nearest point on the next lane in that
                # direction (Confluence routes it round shops/racks via the nearest cross-link)
                ux, uy = math.cos(heading), math.sin(heading)
                cands = []
                for m, (mx, my) in site.NODES.items():
                    if m in site.CHARGERS:
                        continue
                    along = (mx - bx) * ux + (my - by) * uy
                    side = abs(-(mx - bx) * uy + (my - by) * ux)
                    if along > 0.5:
                        cands.append((round(along, 1), side, m))
                if not cands:
                    raise ValueError(f"{r.id} is already at the {d_full} edge of the map")
                best = min(cands)[2]
            node = self.goto(rid, best, _chain=True)
            r.last_jog = d
            return node

    def stop_robot(self, rid):
        """Stop at the next reserved node (smooth). For an instant stop use pause()."""
        with self.lock:
            r = self._robot(rid)
            self._release_task(r, "operator stop")
            self._trim_commitment(r, keep_if_left=PASS_RADIUS)   # stop at the next node at most
            r.last_jog = None
            r.path = list(r.ahead)
            r.goal_kind = "manual" if r.ahead else None
            r.manual_goal = r.ahead[-1] if r.ahead else r.current
            if r.mode != "manual":
                r.mode = "manual"
                self.log.event("robot_mode", robot=rid, mode="manual")
            self.log.event("manual_stop", robot=rid, stopping_at=r.manual_goal)
            return r.manual_goal

    def send_home(self, rid):
        """Send a robot back to its charger; it rejoins automatic work there."""
        with self.lock:
            r = self._robot(rid)
            if not r.home:
                raise ValueError(f"{rid} has no home charger")
            self._release_task(r, "sent home")
            r.mode = "auto"
            r.manual_goal = None
            r.last_jog = None
            r.home_first = True
            self._trim_commitment(r)
            self._plan(r, r.home, "home")
            self.log.event("manual_home", robot=rid, charger=r.home)
            return r.home

    def assign_task(self, task_id, rid):
        """Give a queued task to a specific robot instead of letting Confluence choose."""
        with self.lock:
            r = self._robot(rid)
            t = next((t for t in self.tasks if t.task_id == task_id), None)
            if t is None:
                raise ValueError(f"unknown task {task_id}")
            if t.status != "QUEUED":
                raise ValueError(f"{task_id} is already {t.status}")
            if t.capability not in r.a.capabilities:
                raise ValueError(f"{rid} can't do '{t.capability}' tasks (it can: {', '.join(sorted(r.a.capabilities))})")
            if r.task:
                raise ValueError(f"{rid} is busy with {r.task.task_id}")
            if r.current is None:
                raise ValueError(f"{rid} has not reported its position yet")
            r.mode = "auto"
            r.manual_goal = None
            t.status, t.robot_id, t.assigned_at = "TO_PICKUP", r.id, time.time()
            r.task = t
            self.log.event("task_assigned", task=t.task_id, robot=r.id, protocol=r.a.protocol, by="operator")
            self._plan(r, site.STATIONS[t.pickup], "task")
            return True

    def cancel_task(self, task_id):
        with self.lock:
            t = next((t for t in self.tasks if t.task_id == task_id), None)
            if t is None:
                raise ValueError(f"unknown task {task_id}")
            if t.status in ("DONE", "FAILED"):
                return False
            if t.robot_id:
                r = self.robots[t.robot_id]
                self._set_escort(r, False)
                r.task = None
                r.dwell_until = r.dwell_started = None
                r.work_label = None
                r.path = list(r.ahead)
                r.goal_kind = "task" if r.ahead else None
            t.status, t.finished_at = "FAILED", time.time()
            self.log.event("task_cancelled", task=task_id, robot=t.robot_id, by="operator")
            return True

    # ================================================================ reservations
    def _reserve(self, node, rid):
        owner = self.reserved.get(node)
        if owner is None or owner == rid:
            self.reserved[node] = rid
            return True
        return False

    def _release(self, node, rid):
        if self.reserved.get(node) == rid:
            del self.reserved[node]

    # ================================================================ main loop
    def tick(self):
        with self.lock:
            snaps = {rid: r.a.snapshot() for rid, r in self.robots.items()}
            for rid, r in self.robots.items():
                self._track(r, snaps[rid])
            self.log.check_compliance(snaps, {rid: r.a for rid, r in self.robots.items()})
            if self.auto_tasks and time.time() >= self.next_auto:
                self._auto_task()
            self._assign(snaps)
            for rid, r in self.robots.items():
                self._advance(r, snaps[rid])
            self._resolve_deadlocks(snaps)

    def _track(self, r, s):
        online = s.state != OFFLINE
        if online and not r.was_online:
            self.log.event("link_up" if r.current is None else "link_restored", robot=r.id,
                           protocol=r.a.protocol, telemetry_hz=s.telemetry_hz)
            r.sent_until = None if r.current is None else r.sent_until
            r.last_send = 0.0                     # re-send current command on restore
        elif not online and r.was_online:
            self.log.event("link_lost", robot=r.id, holding_nodes=list(r.held))
        r.was_online = online
        if not online:
            return
        if s.safety_stop and not r.was_safety:
            self.log.event("vendor_safety_stop", robot=r.id, note="robot's own safety system; not overridden")
        r.was_safety = s.safety_stop
        if r._last_xy:
            r.odometer += math.hypot(s.x - r._last_xy[0], s.y - r._last_xy[1])
        r._last_xy = (s.x, s.y)
        if r.task:
            r.busy_time += 0.1
        if r.current is None:                     # first contact: localize on the roadmap
            node, d = site.nearest_node(s.x, s.y)
            if self._reserve(node, r.id):
                r.current = node
                self.log.event("robot_registered", robot=r.id, vendor=s.vendor, model=s.model,
                               protocol=r.a.protocol, node=node, offset_m=round(d, 2))

    # ---------------------------------------------------------------- tasks
    def _auto_task(self):
        self.next_auto = time.time() + self.task_interval
        queued = [t for t in self.tasks if t.status == "QUEUED"]
        if len(queued) >= 3:
            return
        caps = set().union(*(r.a.capabilities for r in self.robots.values()))
        busy_caps = {t.capability for t in self.tasks if t.status not in ("DONE", "FAILED")}
        kinds = [k for k, (c, _, _) in site.TASK_TEMPLATES.items() if c in caps]
        free = [k for k in kinds if site.TASK_TEMPLATES[k][0] not in busy_caps] or kinds
        if free:
            self.add_task(self.rng.choice(free), source="auto")

    def _assign(self, snaps):
        for t in sorted((t for t in self.tasks if t.status == "QUEUED"), key=lambda t: (-t.priority, t.created)):
            best, best_cost = None, 1e9
            for r in self.robots.values():
                s = snaps[r.id]
                if (r.task or r.mode != "auto" or r.home_first or r.operator_paused
                        or (r.rest_until and time.time() < r.rest_until) or s.state in (OFFLINE, ERROR) or r.current is None
                        or t.capability not in r.a.capabilities or s.battery < MIN_BATTERY or r.dwell_until):
                    continue
                path = site.shortest_path(r.current, site.STATIONS[t.pickup])
                if not path:
                    continue
                cost = site.path_length(path) + (15.0 if s.battery < 50 else 0.0)
                if cost < best_cost:
                    best, best_cost = r, cost
            if best:
                t.status, t.robot_id, t.assigned_at = "TO_PICKUP", best.id, time.time()
                best.task = t
                self.log.event("task_assigned", task=t.task_id, robot=best.id, protocol=best.a.protocol,
                               est_distance_m=round(best_cost, 1))
                self._plan(best, site.STATIONS[t.pickup], "task")

    # ---------------------------------------------------------------- planning
    def _immobile(self, o):
        """True if robot o won't move out of the way by itself (parked by the
        operator in manual mode, paused, or offline)."""
        return ((o.mode == "manual" and not o.path and not o.ahead) or o.operator_paused
                or o.a.snapshot().state == OFFLINE)

    def _parked_nodes(self, rid):
        return {n for o in self.robots.values() if o.id != rid and self._immobile(o) for n in o.held}

    def _plan(self, r, goal, kind, avoid=()):
        others = {n for n, o in self.reserved.items() if o != r.id}
        penalty = {n: 4.0 for n in others}
        penalty.update({n: 60.0 for n in self._parked_nodes(r.id)})   # route round parked robots
        start = r.ahead[-1] if r.ahead else r.current
        path = site.shortest_path(start, goal, blocked=avoid, penalty=penalty)
        if path is None:
            return False
        prefix = r.ahead[:]           # keep what the robot is already committed to
        r.path = prefix + path[1:]
        r.goal_kind = kind
        return True

    def _goal_reached(self, r, s):
        node = r.current
        if r.goal_kind == "manual":
            r.goal_kind = None
            self.log.event("manual_goal_reached", robot=r.id, node=node, x=round(s.x, 2), y=round(s.y, 2))
            return
        if r.goal_kind == "home":
            r.goal_kind = None
            r.home_first = False
            if node == r.home:
                self.log.event("robot_docked", robot=r.id, charger=node, battery=round(s.battery, 1))
            return
        t = r.task
        if t is None:
            r.goal_kind = None
            return
        if t.status == "TO_PICKUP":
            t.status = "PICKING"
            self._start_dwell(r, "pick")
        elif t.status == "TO_DROPOFF":
            t.status = "DROPPING"
            self._start_dwell(r, "drop")

    def _work(self, t, phase):
        """(label, seconds) for the pick/drop phase of a task. The time varies a bit
        from job to job, but is fixed per task so every component agrees on it."""
        label, secs = site.TASK_WORK.get(t.kind, {}).get(phase, (phase + "ing", DWELL_S))
        jitter = random.Random(f"{t.task_id}-{phase}").uniform(0.8, 1.25)
        return label, round(secs * jitter, 1)

    def _set_escort(self, r, on):
        if r.escorting != on:
            r.escorting = on
            delivered = r.a.signal("escort", on)
            self.log.event("escort_started" if on else "escort_ended", robot=r.id,
                           task=r.task.task_id if r.task else None, delivered=delivered)

    def _start_dwell(self, r, what):
        label, secs = self._work(r.task, what)
        r.work_label, r.dwell_started, r.dwell_total = label, time.time(), secs
        # a robot doing the action itself reports when it's done; allow it some slack
        r.dwell_until = time.time() + secs + (15.0 if r.native_action else 0.0)
        if what == "pick" and site.TASK_WORK.get(r.task.kind, {}).get("escort"):
            self._set_escort(r, True)            # passenger joins while the robot greets them
        self.log.event(f"{what}_started", robot=r.id, task=r.task.task_id, node=r.current,
                       doing=label, seconds=secs,
                       mode="native VDA 5050 action" if r.native_action else "emulated by Confluence")

    def _dwell_done(self, r, s):
        if time.time() >= r.dwell_until:
            return True
        return r.native_action and s.action_done and time.time() >= r.dwell_started + 1.0

    def _finish_dwell(self, r):
        took = round(time.time() - (r.dwell_started or time.time()), 1)
        r.dwell_until = r.dwell_started = None
        r.native_action = False
        t = r.task
        if t.status == "PICKING":
            t.status = "TO_DROPOFF"
            r.work_label = site.TASK_WORK.get(t.kind, {}).get("work")
            self.log.event("pick_done", robot=r.id, task=t.task_id, took_s=took)
            self._plan(r, site.STATIONS[t.dropoff], "task")
        elif t.status == "DROPPING":
            t.status, t.finished_at = "DONE", time.time()
            self._set_escort(r, False)
            self.log.event("task_completed", robot=r.id, task=t.task_id, kind=t.kind,
                           duration_s=round(t.finished_at - t.assigned_at, 1))
            r.task = None
            r.goal_kind = None
            r.work_label = None
            lo, hi = site.REST_AFTER_TASK
            if hi > 0:
                r.rest_until = time.time() + self.rng.uniform(lo, hi)
                self.log.event("resting", robot=r.id, seconds=round(r.rest_until - time.time(), 1),
                               note="pause after finishing a job")

    # ---------------------------------------------------------------- motion
    def _advance(self, r, s):
        if r.current is None or s.state == OFFLINE or r.operator_paused:
            return
        if s.state == ERROR:
            return
        pos = (s.x, s.y)

        # progress along reserved nodes
        while r.ahead:
            nx, ny = site.NODES[r.ahead[0]]
            final = len(r.ahead) == 1 and r.path == r.ahead
            radius = STOP_RADIUS if final else PASS_RADIUS
            if math.hypot(pos[0] - nx, pos[1] - ny) < radius:
                r.behind.append(r.current)
                r.current = r.ahead.pop(0)
                if r.path and r.path[0] == r.current:
                    r.path.pop(0)
            else:
                break
        docked = r.current in site.CHARGERS and math.hypot(
            pos[0] - site.NODES[r.current][0], pos[1] - site.NODES[r.current][1]) < STOP_RADIUS
        for n in list(r.behind):
            bx, by = site.NODES[n]
            # a docked robot sits fully inside its charger bay (checked by the site geometry),
            # so it can give the lane node back even though it's closer than RELEASE_DIST
            if math.hypot(pos[0] - bx, pos[1] - by) > RELEASE_DIST or docked:
                r.behind.remove(n)
                if n not in r.ahead and n != r.current:
                    self._release(n, r.id)

        # pick/drop in progress
        if r.dwell_until:
            if self._dwell_done(r, s):
                self._finish_dwell(r)
            return

        # arrived at final goal
        if not r.path and not r.ahead and r.goal_kind:
            if r.resume_goal:                       # was pulled aside: carry on to the real goal
                goal, r.resume_goal = r.resume_goal, None
                self._plan(r, goal, r.goal_kind)
                return
            if s.speed < 0.08 or s.state != "MOVING":
                self._goal_reached(r, s)
            return

        # idle -> go home to charger / park
        if r.rest_until and time.time() >= r.rest_until:
            r.rest_until = None
        if not r.path and not r.task and r.goal_kind is None and r.mode == "auto" and not r.rest_until:
            if r.home and r.current != r.home:
                if self._plan(r, r.home, "home"):
                    self.log.event("returning_home", robot=r.id, charger=r.home, battery=round(s.battery, 1))
            return

        # reserve more nodes ahead (straight segments only)
        newly = []
        anchor = r.ahead[-1] if r.ahead else r.current
        remaining = r.path[len(r.ahead):]
        prev = r.ahead[-2] if len(r.ahead) >= 2 else (r.current if r.ahead else None)
        for n in remaining:
            if len(r.ahead) >= LOOKAHEAD:
                break
            if r.ahead and prev is not None and not site.collinear(prev, anchor, n):
                break                                   # stop at the corner, turn there
            if not self._reserve(n, r.id):
                owner = self.reserved.get(n)
                if r.waiting_for != owner:
                    r.waiting_for, r.wait_since = owner, time.time()
                    self.log.event("waiting_for_robot", robot=r.id, blocked_by=owner, node=n)
                break
            r.ahead.append(n)
            newly.append(n)
            prev, anchor = anchor, n
        if newly and r.waiting_for:
            self.log.event("conflict_cleared", robot=r.id, was_blocked_by=r.waiting_for,
                           waited_s=round(time.time() - r.wait_since, 1))
            r.waiting_for = r.wait_since = None

        if not r.ahead:
            return
        if newly or (time.time() - r.last_send > RESEND_S and r.sent_until != r.ahead[-1]):
            self._send_drive(r, s, newly or r.ahead)

    def _speed_for(self, r, nodes):
        lim = None
        for n in [r.current] + list(nodes):
            z = site.zone_limit(*site.NODES[n])
            if z is not None:
                lim = z if lim is None else min(lim, z)
        if r.task and r.task.status == "TO_DROPOFF":          # working speed while doing the job
            ws = site.TASK_WORK.get(r.task.kind, {}).get("speed")
            if ws:
                lim = ws if lim is None else min(lim, ws)
        return lim

    def _send_drive(self, r, s, nodes):
        full = [r.current] + r.ahead
        extend = r.sent_until is not None and r.sent_until in full and r.sent_until != r.ahead[-1]
        if extend:
            nodes = full[full.index(r.sent_until) + 1:]   # only what the robot hasn't been given yet
        else:
            nodes = list(r.ahead)
        start_id = r.current
        sx, sy = site.NODES[start_id]
        lim = self._speed_for(r, nodes)
        wps = [{"node_id": n, "x": site.NODES[n][0], "y": site.NODES[n][1], "speed": lim} for n in nodes]
        action = None
        final = r.ahead[-1] == (r.path[-1] if r.path else None)
        if final and r.task and r.a.supports_native_actions:
            action = "pick" if r.task.status == "TO_PICKUP" else "drop" if r.task.status == "TO_DROPOFF" else None
        secs = self._work(r.task, action)[1] if action else None
        ok = r.a.drive({"node_id": start_id, "x": sx, "y": sy}, wps, extend=extend, action=action,
                       action_seconds=secs)
        r.last_send = time.time()
        if ok:
            r.sent_until = r.ahead[-1]
            r.native_action = action is not None
            if lim is not None and not r.a.supports_speed_limit:
                self.log.event("speed_limit_unsupported", robot=r.id, protocol=r.a.protocol, limit=lim,
                               note="interface cannot carry a speed cap; monitoring only", once_key=f"{r.id}-nolimit")

    # ---------------------------------------------------------------- deadlocks
    def _resolve_deadlocks(self, snaps):
        for r in self.robots.values():
            if not r.waiting_for or r.ahead:
                continue                                 # only robots stopped at their last node
            cycle, cur, seen = None, r, []
            while cur and cur.waiting_for and cur.id not in seen:
                seen.append(cur.id)
                cur = self.robots.get(cur.waiting_for)
                if cur and cur.id == r.id:
                    cycle = seen
            waited = time.time() - (r.wait_since or time.time())
            if cycle:
                loser = min((self.robots[i] for i in cycle), key=lambda x: (x.priority, x.id))
                if loser.ahead or time.time() - loser.last_resolve < 3.0:
                    continue
                loser.last_resolve = time.time()
                others = [self.robots[i] for i in cycle if i != loser.id]
                avoid = {n for o in others for n in o.held}
                goal = loser.resume_goal or (loser.path[-1] if loser.path else None)
                if not goal:
                    continue
                alt = site.shortest_path(loser.current, goal, blocked=avoid)
                if alt and goal not in avoid:
                    loser.path = alt[1:]
                    how = "re-routed around"
                else:
                    # the conflict is at the goal itself: pull aside into a free node, then resume
                    aside = self._find_aside(loser, others)
                    if not aside:
                        self.log.event("deadlock_unresolved", robots=cycle, once_key=f"dl-{'-'.join(sorted(cycle))}")
                        continue
                    loser.resume_goal = goal
                    loser.path = aside
                    how = "pulled aside for"
                self.log.event("deadlock_resolved", robots=cycle, yielding=loser.id,
                               action=f"{loser.id} {how} {', '.join(o.id for o in others)}",
                               rule="lowest priority yields", new_path=list(loser.path))
                loser.waiting_for = loser.wait_since = None
            elif waited > 20.0 and r.path:
                blocker = self.robots.get(r.waiting_for)
                if blocker and blocker.mode == "manual" and not blocker.path:
                    self.log.event("blocked_by_parked_robot", robot=r.id, parked=blocker.id, node=blocker.current,
                                   hint=f"{blocker.id} is in manual mode and parked; move it or send it home",
                                   once_key=f"park-{r.id}-{blocker.id}-{blocker.current}")
                # long wait: look for a detour. Behind a busy robot, only take it if not much
                # longer; if the blocker won't move by itself (parked in manual, paused by the
                # operator, or offline), take any detour - like driving round a parked car.
                avoid = ({n for n in blocker.held} if blocker else set()) | self._parked_nodes(r.id)
                stuck = blocker is not None and self._immobile(blocker)
                if stuck and r.path[-1] in blocker.held:
                    # the destination itself is occupied by a parked robot: no detour can help
                    self.log.event("goal_occupied", robot=r.id, goal=r.path[-1], parked=blocker.id,
                                   hint=f"move {blocker.id} or send {r.id} elsewhere",
                                   once_key=f"goalocc-{r.id}-{r.path[-1]}-{blocker.id}")
                    r.wait_since = time.time()
                    continue
                alt = site.shortest_path(r.current, r.path[-1], blocked=avoid)
                limit = float("inf") if stuck else 1.6 * site.path_length([r.current] + r.path)
                if alt and site.path_length(alt) < limit:
                    r.path = alt[1:]
                    self.log.event("rerouted", robot=r.id, reason="blocker not moving" if stuck else "long wait",
                                   around=blocker.id if blocker else None, waited_s=round(waited, 1))
                r.wait_since = time.time()

    def _find_aside(self, r, others):
        """Nearest free node (BFS, <= 4 hops) off the other robots' next moves."""
        busy = set(self.reserved)
        for o in others:
            busy.update(o.path[:4])
        frontier, seen = [[r.current]], {r.current}
        while frontier:
            path = frontier.pop(0)
            if len(path) > 5:
                break
            for m in sorted(site.EDGES[path[-1]]):
                if m in seen or m in site.CHARGERS or (self.reserved.get(m) not in (None, r.id)):
                    continue
                seen.add(m)
                np = path + [m]
                if m not in busy:
                    return np[1:]
                frontier.append(np)
        return None

    # ================================================================ snapshot
    def snapshot(self):
        with self.lock:
            robots = []
            for rid, r in self.robots.items():
                s = r.a.snapshot()
                d = s.to_dict()
                d.update({
                    "info": r.a.info(), "priority": r.priority, "home": r.home,
                    "current_node": r.current, "path": list(r.path), "reserved_ahead": list(r.ahead),
                    "waiting_for": r.waiting_for, "operator_paused": r.operator_paused,
                    "task": r.task.task_id if r.task else None,
                    "task_status": r.task.status if r.task else None,
                    "goal": r.goal_kind, "odometer_m": round(r.odometer, 1),
                    "mode": r.mode, "manual_goal": r.manual_goal, "home_first": r.home_first,
                    "work": r.work_label, "escorting": r.escorting,
                    "work_left_s": (max(0.0, round(r.dwell_started + r.dwell_total - time.time(), 1))
                                    if r.dwell_started else None),
                    "work_total_s": r.dwell_total if r.dwell_started else None,
                    "rest_left_s": (max(0.0, round(r.rest_until - time.time(), 1)) if r.rest_until else None),
                    "tx": r.a.tx_count, "rx": r.a.rx_count,
                })
                robots.append(d)
            tasks = [t.to_dict() for t in self.tasks[-40:]][::-1]
            done = [t for t in self.tasks if t.status == "DONE"]
            return {
                "time": time.time(), "uptime_s": round(time.time() - self.started, 1),
                "robots": robots, "tasks": tasks,
                "reservations": dict(self.reserved),
                "estopped": self.estopped, "auto_tasks": self.auto_tasks,
                "metrics": {
                    "tasks_done": len(done),
                    "tasks_open": len([t for t in self.tasks if t.status not in ("DONE", "FAILED")]),
                    "avg_task_s": round(sum(t.finished_at - t.assigned_at for t in done) / len(done), 1) if done else None,
                    **self.log.counters(),
                },
            }
