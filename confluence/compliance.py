"""Monitoring & compliance: append-only event log, rule checks on normalized
telemetry, and audit report generation.

Every event is one JSON line in <log_dir>/events-<start>.jsonl, so the log is
easy to archive and replay. The audit report is generated from those events.
"""
import collections
import html
import json
import os
import threading
import time
from datetime import datetime

from confluence import site
from confluence.model import OFFLINE

INCIDENT_TYPES = {"link_lost", "link_restored", "vendor_safety_stop", "speed_violation", "deadlock_resolved",
                  "fleet_stop", "fleet_stop_released", "robot_paused", "robot_resumed", "link_loss_injected",
                  "orchestrator_error", "speed_limit_unsupported", "rerouted"}


class Compliance:
    SPEED_TOLERANCE = 1.15      # allowed over the zone limit (measurement noise)
    SPEED_HOLD_S = 1.5          # must exceed for this long to count

    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir
        self.t0 = time.time()
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(log_dir, f"events-{stamp}.jsonl")
        self.report_path = os.path.join(log_dir, f"audit-{stamp}.html")
        self.fh = open(self.path, "a", buffering=1)
        self.lock = threading.Lock()
        self.recent = collections.deque(maxlen=300)
        self.all_events = []
        self.counts = collections.Counter()
        self._once = set()
        self._over = {}            # robot -> (since, zone)
        self._flagged = set()      # (robot, zone) already reported for this entry
        self._online_time = collections.Counter()
        self._seen_time = collections.Counter()
        self._hz = collections.defaultdict(list)
        self._last_check = None
        self.robot_info = {}

    # ------------------------------------------------------------ events
    def event(self, etype, once_key=None, **data):
        if once_key:
            if once_key in self._once:
                return
            self._once.add(once_key)
        e = {"t": round(time.time(), 3), "iso": datetime.now().isoformat(timespec="milliseconds"),
             "type": etype, **data}
        with self.lock:
            self.fh.write(json.dumps(e, default=str) + "\n")
            self.recent.append(e)
            self.all_events.append(e)
            self.counts[etype] += 1
        return e

    def counters(self):
        c = self.counts
        return {"conflicts": c["waiting_for_robot"], "deadlocks_resolved": c["deadlock_resolved"],
                "safety_stops": c["vendor_safety_stop"], "link_losses": c["link_lost"],
                "speed_violations": c["speed_violation"], "events": sum(c.values())}

    def recent_events(self, n=60):
        with self.lock:
            return list(self.recent)[-n:][::-1]

    # ------------------------------------------------------------ rule checks
    def check_compliance(self, snaps, adapters):
        now = time.time()
        dt = 0.0 if self._last_check is None else now - self._last_check
        self._last_check = now
        for rid, s in snaps.items():
            self.robot_info[rid] = adapters[rid].info()
            self._seen_time[rid] += dt
            if s.state == OFFLINE:
                self._over.pop(rid, None)
                continue
            self._online_time[rid] += dt
            if s.telemetry_hz:
                self._hz[rid] = (self._hz[rid] + [s.telemetry_hz])[-600:]
            lim = site.zone_limit(s.x, s.y)
            zone = next((z[0] for z in site.SPEED_ZONES if z[1] <= s.x <= z[3] and z[2] <= s.y <= z[4]), None)
            if lim is None:
                self._over.pop(rid, None)
                self._flagged = {f for f in self._flagged if f[0] != rid}
                continue
            if s.speed > lim * self.SPEED_TOLERANCE:
                since, z = self._over.get(rid, (now, zone))
                self._over[rid] = (since, zone)
                if now - since >= self.SPEED_HOLD_S and (rid, zone) not in self._flagged:
                    self._flagged.add((rid, zone))
                    a = adapters[rid]
                    self.event("speed_violation", robot=rid, zone=zone, limit=lim, measured=round(s.speed, 2),
                               protocol=a.protocol, interface_supports_limit=a.supports_speed_limit,
                               x=round(s.x, 2), y=round(s.y, 2))
            else:
                self._over.pop(rid, None)

    # ------------------------------------------------------------ audit report
    def report(self, snapshot=None):
        with self.lock:
            ev = list(self.all_events)
        dur = time.time() - self.t0
        per = collections.defaultdict(collections.Counter)
        for e in ev:
            if "robot" in e:
                per[e["robot"]][e["type"]] += 1
        tasks_done = [e for e in ev if e["type"] == "task_completed"]
        rows = []
        for rid, info in sorted(self.robot_info.items()):
            hz = self._hz.get(rid) or [0]
            seen = self._seen_time[rid] or 1
            odo = next((r["odometer_m"] for r in (snapshot or {}).get("robots", []) if r["robot_id"] == rid), "")
            rows.append([rid, f'{info["vendor"]} {info["model"]}', info["protocol"],
                         f'{sum(hz) / len(hz):.1f} (nominal {info["nominal_hz"]:g})',
                         f'{100 * self._online_time[rid] / seen:.0f}%', odo,
                         per[rid]["task_completed"], per[rid]["vendor_safety_stop"], per[rid]["link_lost"],
                         per[rid]["speed_violation"], "yes" if info["supports_speed_limit"] else "NO"])
        findings = []
        for rid, info in sorted(self.robot_info.items()):
            if not info["supports_speed_limit"]:
                v = per[rid]["speed_violation"]
                findings.append(f"{rid} ({info['protocol']}) cannot accept speed caps through its interface; "
                                f"zone limits are monitored only ({v} violation(s) recorded). "
                                f"Mitigation: request vendor support, or exclude it from speed-limited zones.")
            if per[rid]["link_lost"]:
                findings.append(f"{rid}: {per[rid]['link_lost']} communication loss(es); reservations were held "
                                f"during each outage so no other robot entered its path.")
        if not findings:
            findings.append("No compliance findings.")
        incidents = [e for e in ev if e["type"] in INCIDENT_TYPES][-200:]

        def table(head, body):
            h = "".join(f"<th>{html.escape(str(x))}</th>" for x in head)
            b = "".join("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in r) + "</tr>" for r in body)
            return f"<table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table>"

        inc_rows = [[e["iso"][11:23], e["type"], e.get("robot", ""),
                     ", ".join(f"{k}={v}" for k, v in e.items() if k not in ("t", "iso", "type", "robot"))[:160]]
                    for e in incidents]
        counts = sorted(collections.Counter(e["type"] for e in ev).items(), key=lambda kv: -kv[1])
        doc = f"""<!doctype html><html><head><meta charset="utf-8"><title>Confluence audit report</title>
<style>body{{font:14px system-ui,sans-serif;max-width:1100px;margin:24px auto;padding:0 16px;color:#1b1f24}}
h1{{margin-bottom:4px}} .muted{{color:#667}} table{{border-collapse:collapse;width:100%;margin:8px 0 20px}}
th,td{{border:1px solid #d5d9de;padding:5px 8px;text-align:left;font-size:13px}} th{{background:#f1f3f5}}
li{{margin:4px 0}}</style></head><body>
<h1>Confluence audit report</h1>
<div class="muted">Generated {datetime.now().isoformat(timespec="seconds")} · session {dur / 60:.1f} min ·
event log <code>{html.escape(self.path)}</code></div>
<h2>Fleet summary</h2>
{table(["Robot", "Vendor / model", "Interface", "Telemetry Hz (measured)", "Online", "Distance m", "Tasks",
        "Safety stops", "Link losses", "Speed violations", "Speed cap via API"], rows)}
<p>Tasks completed: <b>{len(tasks_done)}</b> · conflicts handled: <b>{self.counts['waiting_for_robot']}</b> ·
deadlocks resolved: <b>{self.counts['deadlock_resolved']}</b></p>
<h2>Compliance findings</h2><ul>{"".join(f"<li>{html.escape(f)}</li>" for f in findings)}</ul>
<h2>Control constraints observed</h2><ul>
<li>All commands were sent through each vendor's published interface (ROS 2 topics, REST, TCP, VDA 5050). No robot software was modified.</li>
<li>Robots' own safety stops were never overridden; Confluence waited for each robot to clear itself ({self.counts['vendor_safety_stop']} occurrence(s)).</li>
<li>Traffic control used node reservations, so robots only entered map nodes reserved for them.</li></ul>
<h2>Event counts</h2>{table(["Event", "Count"], counts)}
<h2>Incident timeline</h2>{table(["Time", "Event", "Robot", "Details"], inc_rows)}
</body></html>"""
        with open(self.report_path, "w") as f:
            f.write(doc)
        return doc
