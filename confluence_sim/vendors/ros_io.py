"""Gazebo-backed robot body: the vendor firmware drives the Gazebo model via
ros_gz_bridge topics (/<name>/cmd_vel, /<name>/odom, /<name>/scan)."""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class GazeboBody:
    FRONT_HALF_ANGLE = math.radians(25)

    def __init__(self, node, name, spawn_xy):
        self.name = name
        self.sx, self.sy = spawn_xy
        self.x, self.y, self.yaw = self.sx, self.sy, 0.0
        self.have_odom = False
        self.front_gap = None
        self.pub = node.create_publisher(Twist, f"/{name}/cmd_vel", 10)
        node.create_subscription(Odometry, f"/{name}/odom", self._odom, 10)
        node.create_subscription(LaserScan, f"/{name}/scan", self._scan, 5)

    def _odom(self, m):
        # DiffDrive odometry starts at the spawn pose (spawned with yaw 0)
        self.x = self.sx + m.pose.pose.position.x
        self.y = self.sy + m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def _scan(self, m):
        best = None
        a = m.angle_min
        for r in m.ranges:
            if abs(a) <= self.FRONT_HALF_ANGLE and m.range_min < r < m.range_max:
                best = r if best is None else min(best, r)
            a += m.angle_increment
        self.front_gap = best

    def pose(self):
        return self.x, self.y, self.yaw

    def set_cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def obstacle_ahead(self, reverse=False):
        return None if reverse else self.front_gap   # front-only lidar
