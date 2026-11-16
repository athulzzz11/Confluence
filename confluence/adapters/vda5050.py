"""VDA 5050 v2 robots (e.g. Kessler K-Lift 12) over MQTT.

Confluence acts as the VDA 5050 master control: it sends orders (with order
updates as the route is released node by node), edge speed limits and native
pick/drop node actions, and reads the AGV state topic."""
import itertools
import json
import threading
import time
from datetime import datetime, timezone

from confluence.adapters.base import RobotAdapter
from confluence.model import ACTING, CHARGING, ERROR, IDLE, MOVING, PAUSED
from confluence.mqtt_compat import make_client


def _now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class Vda5050Adapter(RobotAdapter):
    protocol = "vda5050"
    supports_speed_limit = True
    supports_native_actions = True
    nominal_hz = 1.0

    def __init__(self, cfg):
        super().__init__(cfg)
        ep = cfg["endpoint"]
        self.host, self.port = ep.get("broker_host", "127.0.0.1"), int(ep.get("broker_port", 1883))
        self.manufacturer, self.serial = ep["manufacturer"], ep["serial_number"]
        self.base = f"{ep.get('interface', 'uagv')}/v2/{self.manufacturer}/{self.serial}"
        self.header = itertools.count(1)
        self.order_seq = itertools.count(1)
        self.action_seq = itertools.count(1)
        self.order_id = None
        self.update_id = 0
        self.last_base = None            # (nodeId, sequenceId, x, y) of last released node
        self.pending_action = None
        self.connection_state = "UNKNOWN"
        self.client = None

    def start(self):
        self.client = make_client(f"confluence-{self.serial}")
        self.client.on_connect = lambda c, *a: c.subscribe([(f"{self.base}/state", 0),
                                                            (f"{self.base}/connection", 1)])
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(1, 5)
        self.client.connect_async(self.host, self.port, keepalive=15)
        self.client.loop_start()

    def stop(self):
        if self.client:
            self.client.loop_stop()

    # ------------------------------------------------------------------ inbound
    def _on_message(self, client, userdata, msg):
        try:
            m = json.loads(msg.payload)
        except ValueError:
            return
        if msg.topic.endswith("/connection"):
            self.connection_state = m.get("connectionState", "UNKNOWN")
            if self.connection_state != "ONLINE":
                with self.lock:
                    self.state.online = False
            return
        p = m.get("agvPosition", {})
        errs = [e.get("errorType", "?") for e in m.get("errors", []) if e.get("errorLevel") == "FATAL"]
        acts = {a["actionId"]: a["actionStatus"] for a in m.get("actionStates", [])}
        action_done = True
        if self.pending_action:
            status = acts.get(self.pending_action)
            action_done = status in ("FINISHED", "FAILED")
        acting = any(s == "RUNNING" for s in acts.values())
        charging = m.get("batteryState", {}).get("charging", False)
        if errs:
            st = ERROR
        elif m.get("paused"):
            st = PAUSED
        elif acting:
            st = ACTING
        elif m.get("driving") or m.get("nodeStates"):
            st = MOVING
        elif charging:
            st = CHARGING
        else:
            st = IDLE
        self._received(x=p.get("x", 0.0), y=p.get("y", 0.0), yaw=p.get("theta", 0.0),
                       speed=m.get("velocity", {}).get("vx", 0.0),
                       battery=m.get("batteryState", {}).get("batteryCharge", 0.0),
                       state=st, charging=charging,
                       safety_stop=m.get("safetyState", {}).get("fieldViolation", False),
                       errors=errs, action_done=action_done, raw=m)

    # ------------------------------------------------------------------ outbound
    def _hdr(self):
        return {"headerId": next(self.header), "timestamp": _now(), "version": "2.0.0",
                "manufacturer": self.manufacturer, "serialNumber": self.serial}

    def _pub(self, topic, payload):
        if not self.link_up() or self.client is None:
            return False
        self.client.publish(f"{self.base}/{topic}", json.dumps(payload), qos=1)
        self.tx_count += 1
        return True

    @staticmethod
    def _node(nid, seq, x, y, actions=()):
        return {"nodeId": nid, "sequenceId": seq, "released": True,
                "nodePosition": {"x": x, "y": y, "mapId": "warehouse", "allowedDeviationXY": 0.1},
                "actions": list(actions)}

    def drive(self, start, waypoints, extend=False, action=None, action_seconds=None):
        if not waypoints or not self.link_up():
            return False
        if extend and self.order_id and self.last_base:
            self.update_id += 1
            nid, seq, bx, by = self.last_base
        else:
            if self.order_id and not self._order_finished():
                # VDA 5050: a vehicle rejects a new order while one is running -> cancel it first
                self._instant("cancelOrder")
                self.pending_action = None
            self.order_id = f"confluence-{next(self.order_seq)}"
            self.update_id = 0
            nid, seq, bx, by = start["node_id"], 0, start["x"], start["y"]
        nodes = [self._node(nid, seq, bx, by)]
        edges = []
        prev = nid
        for i, w in enumerate(waypoints):
            seq += 2
            acts = []
            if action and i == len(waypoints) - 1:
                aid = f"{action}-{next(self.action_seq)}"
                params = [{"key": "duration", "value": action_seconds}] if action_seconds else []
                acts = [{"actionId": aid, "actionType": action, "blockingType": "HARD", "actionParameters": params}]
                self.pending_action = aid
                with self.lock:
                    self.state.action_done = False
            nodes.append(self._node(w["node_id"], seq, w["x"], w["y"], acts))
            edge = {"edgeId": f"{prev}-{w['node_id']}", "sequenceId": seq - 1, "released": True,
                    "startNodeId": prev, "endNodeId": w["node_id"], "actions": []}
            if w.get("speed"):
                edge["maxSpeed"] = w["speed"]
            edges.append(edge)
            prev = w["node_id"]
        last = waypoints[-1]
        self.last_base = (last["node_id"], seq, last["x"], last["y"])
        order = {**self._hdr(), "orderId": self.order_id, "orderUpdateId": self.update_id,
                 "nodes": nodes, "edges": edges}
        return self._pub("order", order)

    def _order_finished(self):
        raw = self.state.raw or {}
        return (raw.get("orderId") == self.order_id and not raw.get("nodeStates")
                and not raw.get("driving") and self.last_base is not None
                and raw.get("lastNodeId") == self.last_base[0])

    def _instant(self, action_type):
        aid = f"{action_type}-{next(self.action_seq)}"
        return self._pub("instantActions", {**self._hdr(), "actions": [
            {"actionId": aid, "actionType": action_type, "blockingType": "HARD", "actionParameters": []}]})

    def cancel(self):
        ok = self._instant("cancelOrder")
        self.last_base = None
        self.order_id = None
        self.pending_action = None
        return ok

    def pause(self):
        return self._instant("startPause")

    def resume(self):
        return self._instant("stopPause")
