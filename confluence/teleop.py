"""Drive robots one at a time from the keyboard, through Confluence.

    ros2 run confluence_sim confluence_teleop          # or: python3 -m confluence.teleop
    python3 -m confluence.teleop --url http://localhost:8080

Unlike raw cmd_vel teleop, every move goes through Confluence: the robot
moves along the roadmap one node per key press, only into nodes reserved
for it, so you can't drive it into another robot, and its own safety
system still applies.

  1-9        select robot
  arrows     move one node (also w a s d)
  g          go to a station (type its name)
  space      stop at the next node
  p          pause / resume (instant)
  h          send home (rejoins automatic work)
  m          toggle manual / auto
  q          quit
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty
import urllib.error
import urllib.request

KEYS = {"\x1b[A": "north", "\x1b[B": "south", "\x1b[C": "east", "\x1b[D": "west",
        "w": "north", "s": "south", "d": "east", "a": "west"}


class Api:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def get(self, path):
        with urllib.request.urlopen(self.url + path, timeout=2) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request(self.url + path, data=json.dumps(body or {}).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read() or b"{}")


def read_key(timeout):
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
    if ch == "\x1b":                                   # arrow keys are 3-byte sequences
        while select.select([sys.stdin], [], [], 0.02)[0]:
            ch += os.read(sys.stdin.fileno(), 1).decode(errors="ignore")
    return ch


def line(s, width=110):
    sys.stdout.write("\r" + s[:width].ljust(width))
    sys.stdout.flush()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Keyboard control of individual robots via Confluence")
    ap.add_argument("--url", default="http://localhost:8080")
    args = ap.parse_args(argv)
    api = Api(args.url)
    try:
        site = api.get("/api/site")
        state = api.get("/api/state")
    except (urllib.error.URLError, OSError):
        print(f"Can't reach Confluence at {args.url} - is it running?")
        return 1
    ids = [r["robot_id"] for r in state["robots"]]
    print(__doc__.split("\n\n", 2)[2])
    print(f"Site: {site['title']} · robots: " + "  ".join(f"[{i + 1}] {rid}" for i, rid in enumerate(ids)))
    print("Stations: " + ", ".join(sorted(k for k in site["stations"] if not k.startswith("chg"))))
    sel = ids[0]
    msg = ""
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = 0.0
        while True:
            k = read_key(0.2)
            res = None
            if k:
                if k in ("q", "\x03"):
                    break
                if k.isdigit() and 0 < int(k) <= len(ids):
                    sel = ids[int(k) - 1]
                    msg = f"selected {sel}"
                elif k in KEYS:
                    res = api.post(f"/api/robot/{sel}/jog", {"direction": KEYS[k]})
                    msg = f"{sel} -> {KEYS[k]} {res.get('node', '')}"
                elif k == " ":
                    res = api.post(f"/api/robot/{sel}/stop")
                    msg = f"{sel} stopping at {res.get('node', '')}"
                elif k == "h":
                    res = api.post(f"/api/robot/{sel}/home")
                    msg = f"{sel} going home ({res.get('node', '')})"
                elif k == "p":
                    r = next(r for r in api.get("/api/state")["robots"] if r["robot_id"] == sel)
                    act = "resume" if r["operator_paused"] else "pause"
                    res = api.post(f"/api/robot/{sel}/{act}")
                    msg = f"{sel} {act}d"
                elif k == "m":
                    r = next(r for r in api.get("/api/state")["robots"] if r["robot_id"] == sel)
                    mode = "auto" if r["mode"] == "manual" else "manual"
                    res = api.post(f"/api/robot/{sel}/mode", {"mode": mode})
                    msg = f"{sel} now {mode}"
                elif k == "g":
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
                    target = input(f"\n{sel} go to station: ").strip()
                    tty.setcbreak(sys.stdin.fileno())
                    if target:
                        res = api.post(f"/api/robot/{sel}/goto", {"target": target})
                        msg = f"{sel} -> {target} ({res.get('node', '')})"
                if res and "error" in res:
                    msg = f"! {res['error']}"
            if time.time() - last > 0.5 or k:
                last = time.time()
                try:
                    r = next(r for r in api.get("/api/state")["robots"] if r["robot_id"] == sel)
                    status = (f"[{sel}] {r['info']['vendor']} via {r['info']['protocol'].upper()} | "
                              f"{r['mode'].upper()} {r['state']} at {r['current_node']} "
                              f"({r['x']:.1f},{r['y']:.1f}) batt {r['battery']:.0f}%")
                    if r["waiting_for"]:
                        status += f" | waiting for {r['waiting_for']}"
                    if r["safety_stop"]:
                        status += " | SAFETY STOP"
                except (urllib.error.URLError, OSError, StopIteration):
                    status = "[connection lost]"
                line(f"{status}   {msg}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
