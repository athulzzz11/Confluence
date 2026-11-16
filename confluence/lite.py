"""Run the whole stack without ROS 2 or Gazebo: the four simulated vendor
robots on a simple kinematic simulator + Confluence + dashboard.
Useful for developing on any laptop, or as a fallback if Gazebo misbehaves.

    python3 -m confluence.lite              # then open http://localhost:8080
    python3 -m confluence.lite --site airport
    python3 -m confluence.lite --duration 300 --no-dashboard   # headless test run
"""
import argparse
import json
import threading
import time

from confluence.core import Confluence, load_config
from confluence.rosbus import LocalBus


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--site", choices=["warehouse", "airport"])
    ap.add_argument("--config")
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = run until Ctrl-C")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-dashboard", action="store_true")
    ap.add_argument("--log-dir")
    args = ap.parse_args(argv)

    from confluence import site
    if args.site:
        site.load(args.site)
    from confluence_sim.vendors.run import run_kinematic   # the simulated vendor robots
    bus = LocalBus()
    stop = threading.Event()
    fws, bodies, _ = run_kinematic(bus, stop)
    time.sleep(0.5)
    app = Confluence(load_config(args.config), ros_bus=bus,
                     dashboard_port=0 if args.no_dashboard else args.port, log_dir=args.log_dir).start()
    if not args.no_dashboard:
        print(f"Confluence dashboard: http://localhost:{args.port}")
    try:
        if args.duration:
            time.sleep(args.duration)
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        pass
    snap = app.orch.snapshot()
    app.stop()
    stop.set()
    print(json.dumps(snap["metrics"], indent=1))
    print("audit report:", app.compliance.report_path)
    return app, snap, bodies


if __name__ == "__main__":
    main()
