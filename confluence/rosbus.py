"""Thin pub/sub layer for ROS 2 topics.

RclpyBus  - real ROS 2 (rclpy). Used whenever ROS 2 is available (Gazebo runs).
LocalBus  - in-process stand-in with the same API, so the full stack can be
            tested on a machine without ROS 2 (`confluence-lite`).

Values are plain Python; the message type is chosen from the topic suffix:
    .../goal_pose    geometry_msgs/PoseStamped   (x, y, yaw)
    .../speed_limit  std_msgs/Float32             float
    anything else    std_msgs/String              str (JSON payloads)
"""
import math
import threading
from collections import defaultdict


def _kind(topic):
    if topic.endswith("goal_pose"):
        return "pose"
    if topic.endswith("speed_limit"):
        return "float"
    return "string"


class LocalBus:
    def __init__(self):
        self.subs = defaultdict(list)
        self.lock = threading.Lock()

    def publish(self, topic, value):
        with self.lock:
            cbs = list(self.subs[topic])
        for cb in cbs:
            cb(value)

    def subscribe(self, topic, cb):
        with self.lock:
            self.subs[topic].append(cb)


class RclpyBus:
    """Wraps an existing rclpy Node. Callbacks run in the node's executor."""

    def __init__(self, node):
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Float32, String
        self.node = node
        self.types = {"pose": PoseStamped, "float": Float32, "string": String}
        self.pubs = {}

    def _to_msg(self, kind, value):
        msg = self.types[kind]()
        if kind == "pose":
            x, y, yaw = value
            msg.header.frame_id = "map"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = float(x), float(y)
            msg.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.orientation.w = math.cos(yaw / 2)
        elif kind == "float":
            msg.data = float(value)
        else:
            msg.data = str(value)
        return msg

    @staticmethod
    def _from_msg(kind, msg):
        if kind == "pose":
            q = msg.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return msg.pose.position.x, msg.pose.position.y, yaw
        return msg.data

    def publish(self, topic, value):
        kind = _kind(topic)
        if topic not in self.pubs:
            self.pubs[topic] = self.node.create_publisher(self.types[kind], topic, 10)
        self.pubs[topic].publish(self._to_msg(kind, value))

    def subscribe(self, topic, cb):
        kind = _kind(topic)
        self.node.create_subscription(self.types[kind], topic,
                                      lambda m: cb(self._from_msg(kind, m)), 10)
