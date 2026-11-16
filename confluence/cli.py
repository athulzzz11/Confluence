"""Command-line control of a running Confluence (warehouse/airport, Gazebo or lite).
Talks to the dashboard API, so it needs no ROS sourcing and works from any terminal.
Normally used through the launcher:  confluence <command> ...

  status                         robots, modes, positions, tasks
  send <robot> <station|x y>     drive a robot somewhere (puts it in MANUAL)
  jog <robot> <n|s|e|w>          move one point; repeat the same direction to keep going
  stop <robot|all>               stop at the next point
  pause <robot|all> / resume <robot|all>
  home <robot|all>               back to charger, back to AUTO
  manual <robot|all> / auto <robot|all>
  task <kind> [pickup] [dropoff] create a task (Confluence picks the robot)
  assign <task> <robot> / cancel <task>
  tasks                          list tasks
  autotasks on|off               automatic task generator
  estop / release                fleet stop for every robot
  stations                       robots, stations and task kinds for this site
"""
import json
import os
import sys
import urllib.error
import urllib.request

URL = os.environ.get("CONFLUENCE_URL", "http://localhost:8080").rstrip("/")
DIRS = {"n": "north", "north": "north", "up": "north", "s": "south", "south": "south", "down": "south",
        "e": "east", "east": "east", "right": "east", "w": "west", "west": "west", "left": "west"}


class Down(Exception):
    pass


def _req(method, path, body=None):
    data = json.dumps(body or {}).encode() if method == "POST" else None
    req = urllib.request.Request(URL + path, data=data, method=method, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=4) as r:
            raw = r.read()
            return json.loads(raw) if raw[:1] in (b"{", b"[") else raw.decode()
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read() or b"{}")
        except ValueError:
            return {"error": f"HTTP {e.code}"}
    except (urllib.error.URLError, OSError):
        raise Down()


def get(path):
    return _req("GET", path)


def post(path, body=None):
    return _req("POST", path, body)


def ok(res, success):
    if isinstance(res, dict) and res.get("error"):
        print(f"  ✗ {res['error']}")
        return False
    print(f"  ✓ {success}")
    return True


def robot_ids():
    return [r["robot_id"] for r in get("/api/state")["robots"]]


def targets(arg):
    ids = robot_ids()
    if arg == "all":
        return ids
    if arg not in ids:
        raise SystemExit(f"  ✗ unknown robot '{arg}' (robots: {', '.join(ids)})")
    return [arg]


def cmd_status(_):
    s = get("/api/state")
    site = get("/api/site")
    m = s["metrics"]
    print(f"\n  {site['title']} · up {int(s['uptime_s'] // 60)}m{int(s['uptime_s'] % 60):02d}s · "
          f"auto tasks {'ON' if s['auto_tasks'] else 'off'}{'  · FLEET STOPPED' if s['estopped'] else ''}")
    print(f"  tasks done {m['tasks_done']} · open {m['tasks_open']} · conflicts {m['conflicts']} · "
          f"deadlocks fixed {m['deadlocks_resolved']}\n")
    print(f"  {'robot':5} {'protocol':9} {'mode':7} {'state':9} {'at':6} {'x,y':>12}  {'batt':>5}  doing")
    for r in s["robots"]:
        doing = ""
        if r["operator_paused"]:
            doing = "paused"
        elif r["waiting_for"]:
            doing = f"waiting for {r['waiting_for']}"
        elif r["task"] and r.get("work_left_s") is not None:
            doing = f"{r['task']} {r['work']} ({r['work_left_s']:.0f}s left)"
        elif r["task"]:
            doing = f"{r['task']} {r.get('work') or r['task_status'].lower().replace('_', ' ')}"
        elif r.get("rest_left_s") is not None:
            doing = f"job done, pausing {r['rest_left_s']:.0f}s"
        elif r["goal"] == "manual":
            doing = f"driving to {r['manual_goal']}"
        elif r["goal"] == "home":
            doing = f"going home ({r['home']})"
        elif r["mode"] == "manual":
            doing = "parked, waiting for you"
        if r["safety_stop"]:
            doing += " [own safety stop]"
        print(f"  {r['robot_id']:5} {r['info']['protocol']:9} {r['mode'].upper():7} {r['state']:9} "
              f"{(r['current_node'] or '-'):6} {r['x']:5.1f},{r['y']:5.1f}  {r['battery']:4.0f}%  {doing}")
    print(f"\n  dashboard: {URL}\n")


def cmd_send(a):
    if len(a) not in (2, 3):
        raise SystemExit("  usage: send <robot> <station>   or   send <robot> <x> <y>")
    rid = targets(a[0])[0]
    if len(a) == 3:
        body, what = {"x": float(a[1]), "y": float(a[2])}, f"({a[1]}, {a[2]})"
    else:
        body, what = {"target": a[1]}, a[1]
    res = post(f"/api/robot/{rid}/goto", body)
    ok(res, f"{rid} → {what} (point {res.get('node')}), now MANUAL")


def cmd_jog(a):
    if len(a) != 2 or a[1].lower() not in DIRS:
        raise SystemExit("  usage: jog <robot> <north|south|east|west>")
    rid = targets(a[0])[0]
    res = post(f"/api/robot/{rid}/jog", {"direction": DIRS[a[1].lower()]})
    ok(res, f"{rid} → {DIRS[a[1].lower()]} to point {res.get('node')}")


def _each(a, action, label, body=None):
    if len(a) != 1:
        raise SystemExit(f"  usage: {action} <robot|all>")
    for rid in targets(a[0]):
        res = post(f"/api/robot/{rid}/{action}", body)
        ok(res, f"{rid} {label}" + (f" ({res['node']})" if isinstance(res, dict) and res.get("node") else ""))


def cmd_task(a):
    if not a:
        raise SystemExit("  usage: task <kind> [pickup] [dropoff]   (see: stations)")
    body = {"kind": a[0], "pickup": a[1] if len(a) > 1 else None, "dropoff": a[2] if len(a) > 2 else None}
    res = post("/api/task", body)
    ok(res, f"{res.get('task_id')} {res.get('kind')}: {res.get('pickup')} → {res.get('dropoff')} queued")


def cmd_tasks(_):
    s = get("/api/state")
    if not s["tasks"]:
        print("  no tasks yet")
    for t in s["tasks"][:20]:
        print(f"  {t['task_id']}  {t['status']:11} {t['kind']:18} {t['pickup']} → {t['dropoff']}  "
              f"{t['robot_id'] or '-'}")


def cmd_stations(_):
    site = get("/api/site")
    s = get("/api/state")
    print(f"\n  {site['title']}")
    print("  robots:   " + "  ".join(f"{r['robot_id']} ({r['info']['vendor']}, {r['info']['protocol']}, "
                                   f"can: {'/'.join(r['info']['capabilities'])})" for r in s["robots"]))
    print("  stations: " + ", ".join(sorted(k for k in site["stations"] if not k.startswith("chg"))))
    print("  tasks:    " + ", ".join(f"{k} ({v['capability']})" for k, v in site["task_templates"].items()))
    print()


def main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if not a or a[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    c, rest = a[0].lower(), a[1:]
    try:
        if c == "status":
            cmd_status(rest)
        elif c in ("send", "goto", "go"):
            cmd_send(rest)
        elif c == "jog":
            cmd_jog(rest)
        elif c == "stop":
            if not rest:
                raise SystemExit("  usage: stop <robot|all>   (to shut everything down: confluence shutdown)")
            _each(rest, "stop", "stopping at the next point")
        elif c == "pause":
            _each(rest, "pause", "paused")
        elif c == "resume":
            _each(rest, "resume", "resumed")
        elif c == "home":
            _each(rest, "home", "going home, AUTO")
        elif c in ("manual", "auto"):
            _each(rest, "mode", f"now {c.upper()}", {"mode": c})
        elif c == "task":
            cmd_task(rest)
        elif c == "tasks":
            cmd_tasks(rest)
        elif c == "assign":
            if len(rest) != 2:
                raise SystemExit("  usage: assign <task> <robot>")
            ok(post(f"/api/task/{rest[0].upper()}/assign", {"robot": rest[1]}), f"{rest[0].upper()} → {rest[1]}")
        elif c == "cancel":
            if len(rest) != 1:
                raise SystemExit("  usage: cancel <task>")
            ok(post(f"/api/task/{rest[0].upper()}/cancel"), f"{rest[0].upper()} cancelled")
        elif c == "autotasks":
            on = (rest[:1] or ["on"])[0].lower() in ("on", "1", "true", "yes")
            ok(post("/api/auto", {"on": on}), f"automatic tasks {'ON' if on else 'OFF'}")
        elif c == "estop":
            ok(post("/api/estop"), "FLEET STOP - every robot paused (confluence release to continue)")
        elif c == "release":
            ok(post("/api/release"), "fleet stop released")
        elif c == "stations":
            cmd_stations(rest)
        elif c == "ping":
            get("/api/state")
        else:
            print(f"  ✗ unknown command '{c}'\n{__doc__}")
            return 2
    except Down:
        print(f"  ✗ Confluence isn't running (nothing answers at {URL}).\n"
              f"    Start it with:  confluence start airport")
        return 3
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:          # output piped into head etc.
        sys.exit(0)
