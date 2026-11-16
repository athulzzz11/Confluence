"""Confluence as a ROS 2 node.

    ros2 run confluence_sim confluence --ros-args -p config:=/path/fleet.yaml
(Without a config, fleet_<site>.yaml is used; the site comes from CONFLUENCE_SITE.)

ROS 2 interface:
  pub /confluence/fleet_state   std_msgs/String  JSON snapshot, 2 Hz
  pub /confluence/events        std_msgs/String  one JSON event per message
  sub /confluence/task_request  std_msgs/String  {"kind": "move_pallet", "pickup"?, "dropoff"?}
  sub /confluence/command       std_msgs/String  fleet-wide or per-robot commands, e.g.
        "goto r1 gate_2" | "goto r1 3.0 -1.4" | "jog r1 north" | "stop r1" | "home r1"
        "mode r1 manual" | "mode r1 auto" | "pause r1" | "resume r1" | "assign T0004 r2"
        "cancel T0004" | "estop" | "release" | "linkloss r2 10" | "auto on|off" | "report"

Per robot (one set for every robot in the fleet config):
  sub /confluence/robots/<id>/goal_pose  geometry_msgs/PoseStamped  drive there (e.g. RViz "2D Goal Pose")
  sub /confluence/robots/<id>/command    std_msgs/String  "goto gate_2" | "jog north" | "stop" | "home"
                                                          | "pause" | "resume" | "manual" | "auto"
  pub /confluence/robots/<id>/state      std_msgs/String  JSON state of that robot, 2 Hz
Robots that speak ROS 2 natively are reached on their own topics through the
same node; the other vendors are reached over REST, TCP and VDA 5050/MQTT.
"""
import json

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from confluence.core import DEFAULT_CONFIG, Confluence, load_config
from confluence.rosbus import RclpyBus


class ConfluenceNode(Node):
    def __init__(self):
        super().__init__("confluence")
        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("dashboard_port", 8080)
        cfg = load_config(self.get_parameter("config").value)
        port = int(self.get_parameter("dashboard_port").value)
        self.app = Confluence(cfg, ros_bus=RclpyBus(self), dashboard_port=port).start()
        self.state_pub = self.create_publisher(String, "/confluence/fleet_state", 10)
        self.event_pub = self.create_publisher(String, "/confluence/events", 50)
        self.create_subscription(String, "/confluence/task_request", self._on_task, 10)
        self.create_subscription(String, "/confluence/command", self._on_command, 10)
        self.robot_state_pubs = {}
        for rid in self.app.orch.robots:
            self.robot_state_pubs[rid] = self.create_publisher(String, f"/confluence/robots/{rid}/state", 10)
            self.create_subscription(PoseStamped, f"/confluence/robots/{rid}/goal_pose",
                                     lambda m, rid=rid: self._on_goal(rid, m), 10)
            self.create_subscription(String, f"/confluence/robots/{rid}/command",
                                     lambda m, rid=rid: self._on_robot_command(rid, m.data), 10)
        self.create_timer(0.5, self._publish_state)
        self.create_timer(0.2, self._publish_events)
        self._event_idx = 0
        from confluence import site
        self.get_logger().info(
            f"Confluence up at {site.TITLE}: {len(cfg['robots'])} robots, "
            f"protocols {sorted({r['protocol'] for r in cfg['robots']})}; dashboard http://localhost:{port}")

    def _publish_state(self):
        snap = self.app.orch.snapshot()
        self.state_pub.publish(String(data=json.dumps(snap, default=str)))
        for r in snap["robots"]:
            pub = self.robot_state_pubs.get(r["robot_id"])
            if pub:
                pub.publish(String(data=json.dumps(r, default=str)))

    def _on_goal(self, rid, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        try:
            node = self.app.orch.goto(rid, (x, y))
            self.get_logger().info(f"{rid}: goal ({x:.2f}, {y:.2f}) -> roadmap node {node}")
        except ValueError as e:
            self.get_logger().warn(f"{rid}: {e}")

    def _on_robot_command(self, rid, text):
        self._on_command(String(data=self._expand(rid, text)))

    @staticmethod
    def _expand(rid, text):
        """'jog north' on /robots/r1/command -> 'jog r1 north'; 'manual' -> 'mode r1 manual'."""
        p = text.split()
        if not p:
            return ""
        if p[0] in ("manual", "auto") and len(p) == 1:
            return f"mode {rid} {p[0]}"
        return " ".join([p[0], rid] + p[1:])

    def _publish_events(self):
        evs = self.app.compliance.all_events
        while self._event_idx < len(evs):
            self.event_pub.publish(String(data=json.dumps(evs[self._event_idx], default=str)))
            self._event_idx += 1

    def _on_task(self, msg):
        try:
            b = json.loads(msg.data)
            t = self.app.orch.add_task(b["kind"], b.get("pickup"), b.get("dropoff"), source="ros")
            self.get_logger().info(f"task {t.task_id} queued from ROS 2: {t.kind} {t.pickup} -> {t.dropoff}")
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f"bad task_request '{msg.data}': {e}")

    def _on_command(self, msg):
        o, p = self.app.orch, msg.data.split()
        try:
            cmd = p[0]
            if cmd == "pause":
                o.pause(p[1])
            elif cmd == "resume":
                o.resume(p[1])
            elif cmd == "goto":
                target = (float(p[2]), float(p[3])) if len(p) == 4 else p[2]
                node = o.goto(p[1], target)
                self.get_logger().info(f"{p[1]} -> {node}")
            elif cmd == "jog":
                self.get_logger().info(f"{p[1]} -> {o.jog(p[1], p[2])}")
            elif cmd == "stop":
                o.stop_robot(p[1])
            elif cmd == "home":
                o.send_home(p[1])
            elif cmd == "mode":
                o.set_mode(p[1], p[2])
            elif cmd == "assign":
                o.assign_task(p[1], p[2])
            elif cmd == "cancel":
                o.cancel_task(p[1])
            elif cmd == "estop":
                o.estop()
            elif cmd == "release":
                o.release_estop()
            elif cmd == "linkloss":
                o.link_loss(p[1], float(p[2]) if len(p) > 2 else 10.0)
            elif cmd == "auto":
                o.set_auto(p[1] == "on")
            elif cmd == "report":
                self.app.compliance.report(o.snapshot())
                self.get_logger().info(f"audit report written to {self.app.compliance.report_path}")
            else:
                raise ValueError(f"unknown command '{cmd}'")
        except (IndexError, KeyError) as e:
            self.get_logger().warn(f"incomplete command '{msg.data}'")
        except ValueError as e:
            self.get_logger().warn(f"'{msg.data}': {e}")


def main():
    rclpy.init()
    node = ConfluenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.app.stop()
        node.get_logger().info(f"audit report: {node.app.compliance.report_path}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
