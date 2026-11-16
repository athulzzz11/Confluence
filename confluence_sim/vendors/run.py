"""Starts the four simulated vendor robots, each with its own API.

    ros2 run confluence_sim vendor_robots            # drives the Gazebo robots
    python3 -m confluence_sim.vendors.run --backend kinematic   # no Gazebo
(The site - warehouse or airport - comes from CONFLUENCE_SITE.)

| id | brand (fictional)       | interface               |
|----|-------------------------|-------------------------|
| r1 | Nimbus Robotics         | ROS 2 topics            |
| r2 | Atlas Mobility          | REST/HTTP  :8081        |
| r3 | Tugwell                 | TCP line protocol :9090 |
| r4 | Kessler Intralogistics  | VDA 5050 over MQTT :1883|

At the airport this process also walks the passengers (vendors/crowd.py).
"""
import argparse
import threading
import time

from confluence_sim.fleet_config import guests, passengers, robots
from confluence_sim.vendors.crowd import Guest, Walker
from confluence_sim.vendors import mini_broker
from confluence_sim.vendors.api_rest import RestApi
from confluence_sim.vendors.api_ros import RosNativeApi
from confluence_sim.vendors.api_tcp import TcpApi
from confluence_sim.vendors.api_vda5050 import Vda5050Api
from confluence_sim.vendors.firmware import Firmware, KinematicBody, KinematicWorld

REST_PORT, TCP_PORT, MQTT_PORT = 8081, 9090, 1883


def build(bodies, bus, host="127.0.0.1"):
    """Wire firmware + vendor API for each robot. Returns (firmwares, apis)."""
    cfg = {r["name"]: r for r in robots()}
    fws = {n: Firmware(n, bodies[n], cfg[n]["max_v"], accepts_speed_limit=(n != "r3")) for n in bodies}
    if mini_broker.start(MQTT_PORT):
        print(f"[vendors] started built-in MQTT broker on :{MQTT_PORT}")
    else:
        print(f"[vendors] using existing MQTT broker on :{MQTT_PORT}")
    apis = {
        "r1": RosNativeApi(fws["r1"], bus, "/r1"),
        "r2": RestApi(fws["r2"], host, REST_PORT),
        "r3": TcpApi(fws["r3"], host, TCP_PORT),
        "r4": Vda5050Api(fws["r4"], host, MQTT_PORT, "Kessler", "KL12-0004"),
    }
    return fws, apis


class ApiTicker:
    """Each vendor publishes at its own native rate (inconsistent telemetry)."""

    def __init__(self, apis):
        self.apis = apis
        self.next = {k: 0.0 for k in apis}

    def tick(self):
        now = time.time()
        for k, api in self.apis.items():
            hz = getattr(api, "rate_hz", None) or getattr(api, "state_hz", None)
            if hz is None:
                continue                      # REST/TCP serve on their own threads
            if now >= self.next[k]:
                api.tick()
                self.next[k] = now + 1.0 / hz
            elif k == "r4":
                api.tick()                    # VDA: publish-on-change check


def run_kinematic(bus, stop_event=None):
    world = KinematicWorld()
    bodies = {}
    for c in robots():
        b = KinematicBody(c["name"], c["spawn"][0], c["spawn"][1], 0.0, c["size"][0], c["size"][1])
        world.add(b)
        bodies[c["name"]] = b
    fws, apis = build(bodies, bus)
    ticker = ApiTicker(apis)
    walkers, people = [], []
    for i, p in enumerate(passengers()):
        pb = KinematicBody(p["name"], p["spawn"][0], p["spawn"][1], 0.0, 0.4, 0.4)
        world.add(pb)             # robots' protective fields see people too
        people.append(pb)
        walkers.append(Walker(p["name"], p["route"], pb, list(bodies.values()), seed=i))
    for i, g in enumerate(guests()):
        gb = KinematicBody(g["name"], g["spawn"][0], g["spawn"][1], 0.0, 0.4, 0.4)
        world.add(gb)
        people.append(gb)
        walkers.append(Guest(g["name"], g["cfg"], gb, fws[g["robot"]], bodies[g["robot"]],
                             [b for n, b in bodies.items() if n != g["robot"]], seed=100 + i))
    stop_event = stop_event or threading.Event()

    def loop():
        dt = 0.05
        while not stop_event.is_set():
            t0 = time.time()
            for fw in fws.values():
                fw.step(dt)
            for w in walkers:
                w.step(dt)
            for b in list(bodies.values()) + people:
                b.step(dt)
            ticker.tick()
            time.sleep(max(0.0, dt - (time.time() - t0)))

    th = threading.Thread(target=loop, daemon=True)
    th.start()
    bodies_all = dict(bodies)
    bodies_all.update({p.name: p for p in people})
    return fws, bodies_all, th


def run_gazebo():
    import rclpy
    from rclpy.node import Node

    from confluence.rosbus import RclpyBus
    from confluence_sim.vendors.ros_io import GazeboBody

    rclpy.init()
    node = Node("vendor_robots")
    bodies = {c["name"]: GazeboBody(node, c["name"], c["spawn"]) for c in robots()}
    fws, apis = build(bodies, RclpyBus(node))
    ticker = ApiTicker(apis)
    people = {p["name"]: GazeboBody(node, p["name"], p["spawn"]) for p in passengers()}
    walkers = [Walker(p["name"], p["route"], people[p["name"]], list(bodies.values()), seed=i)
               for i, p in enumerate(passengers())]
    for i, g in enumerate(guests()):
        people[g["name"]] = GazeboBody(node, g["name"], g["spawn"])
        walkers.append(Guest(g["name"], g["cfg"], people[g["name"]], fws[g["robot"]], bodies[g["robot"]],
                             [b for n, b in bodies.items() if n != g["robot"]], seed=100 + i))
    if walkers:
        node.get_logger().info(f"{len(walkers)} people in the terminal (walkers + guided passenger)")
    node.get_logger().info("vendor robots up: r1 ROS 2 | r2 REST :8081 | r3 TCP :9090 | r4 VDA 5050 MQTT :1883")

    def step():
        for n, fw in fws.items():
            if bodies[n].have_odom:
                fw.step(0.05)
        for w in walkers:
            if w.body.have_odom:
                w.step(0.05)
        ticker.tick()

    node.create_timer(0.05, step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for b in list(bodies.values()) + list(people.values()):
            b.set_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["gazebo", "kinematic"], default="gazebo")
    args, _ = ap.parse_known_args()
    if args.backend == "gazebo":
        run_gazebo()
    else:
        from confluence.rosbus import LocalBus
        print("[vendors] kinematic backend: r1's ROS 2 API is only reachable in-process; "
              "use `confluence-lite` for a full no-Gazebo demo")
        run_kinematic(LocalBus())
        threading.Event().wait()


if __name__ == "__main__":
    main()
