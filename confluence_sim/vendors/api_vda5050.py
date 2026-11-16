"""Vendor 4 - "Kessler Intralogistics K-Lift 12": VDA 5050 v2 over MQTT.

Topics: uagv/v2/<manufacturer>/<serialNumber>/{order, instantActions, state, connection}

Implemented subset of VDA 5050 2.0:
  * order with nodes/edges, base vs horizon (released flag), order updates
    (same orderId, higher orderUpdateId, stitched at the last base node)
  * edge maxSpeed, node actions pick / drop (blockingType HARD)
  * instantActions: startPause, stopPause, cancelOrder
  * state: lastNodeId, lastNodeSequenceId, nodeStates, edgeStates, agvPosition,
    velocity, driving, paused, batteryState, safetyState, errors, actionStates
    published at 1 Hz and immediately on change
  * connection topic: ONLINE (retained) with CONNECTIONBROKEN as last will
"""
import json
import threading
import time
from datetime import datetime, timezone

from confluence.mqtt_compat import make_client

ACTION_DURATION = {"pick": 4.0, "drop": 4.0}


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class Vda5050Api:
    state_hz = 1.0

    def __init__(self, fw, broker_host, broker_port, manufacturer, serial):
        self.fw = fw
        self.base = f"uagv/v2/{manufacturer}/{serial}"
        self.manufacturer, self.serial = manufacturer, serial
        self.header = 0
        self.lock = threading.RLock()
        self.order_id, self.order_update = "", 0
        self.nodes, self.edges = [], []        # remaining (not yet traversed) node/edge dicts
        self.action_states = {}                # actionId -> {"actionId","actionType","actionStatus"}
        self.errors = []
        self._last_sig = None
        self._last_pub = 0.0

        self.client = make_client(f"{serial}-vehicle")
        self.client.will_set(f"{self.base}/connection", self._conn_msg("CONNECTIONBROKEN"), qos=1, retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(1, 5)
        self.client.connect_async(broker_host, broker_port, keepalive=15)
        self.client.loop_start()

    # ------------------------------------------------------------- mqtt
    def _hdr(self):
        self.header += 1
        return {"headerId": self.header, "timestamp": now_iso(), "version": "2.0.0",
                "manufacturer": self.manufacturer, "serialNumber": self.serial}

    def _conn_msg(self, state):
        return json.dumps({**self._hdr(), "connectionState": state})

    def _on_connect(self, client, *args):
        client.subscribe([(f"{self.base}/order", 1), (f"{self.base}/instantActions", 1)])
        client.publish(f"{self.base}/connection", self._conn_msg("ONLINE"), qos=1, retain=True)

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
        except ValueError:
            return
        if msg.topic.endswith("/order"):
            self._handle_order(data)
        elif msg.topic.endswith("/instantActions"):
            for a in data.get("actions", data.get("instantActions", [])):
                self._handle_instant(a)
        self.publish_state(force=True)

    # ------------------------------------------------------------- order handling
    def _goals_from(self, nodes, edges):
        speed_to = {e["endNodeId"]: e.get("maxSpeed") for e in edges if e.get("released", True)}
        goals = []
        for n in nodes:
            if not n.get("released", True):
                break                                    # horizon: planned, not yet allowed
            p = n["nodePosition"]
            acts = []
            for a in n.get("actions", []):
                if a["actionType"] in ACTION_DURATION:
                    secs = next((float(q["value"]) for q in a.get("actionParameters", [])
                                 if q.get("key") == "duration"), ACTION_DURATION[a["actionType"]])
                    acts.append({"type": a["actionType"], "id": a["actionId"], "duration": secs})
                    self.action_states[a["actionId"]] = {"actionId": a["actionId"],
                                                         "actionType": a["actionType"],
                                                         "actionStatus": "WAITING"}
            goals.append({"x": p["x"], "y": p["y"], "node_id": n["nodeId"], "seq": n["sequenceId"],
                          "speed": speed_to.get(n["nodeId"]), "actions": acts})
        return goals

    def _handle_order(self, o):
        with self.lock:
            self.errors = [e for e in self.errors if e["errorType"] != "orderUpdateError"]
            if o["orderId"] == self.order_id:
                if o["orderUpdateId"] <= self.order_update:
                    return                                       # duplicate, ignore (spec)
                # stitching: first node of the update must be our last base node
                base_last = self.fw.goals[-1]["seq"] if self.fw.goals else self.fw.last_seq
                first = o["nodes"][0]["sequenceId"]
                if first != base_last:
                    self.errors.append({"errorType": "orderUpdateError", "errorLevel": "WARNING",
                                        "errorDescription": f"update starts at seq {first}, expected {base_last}"})
                    return
                self.order_update = o["orderUpdateId"]
                new_nodes = [n for n in o["nodes"] if n["sequenceId"] > base_last]
                self.fw.append_goals(self._goals_from(new_nodes, o.get("edges", [])))
            else:
                if self.fw.busy:
                    self.errors.append({"errorType": "orderError", "errorLevel": "WARNING",
                                        "errorDescription": "new order while previous order active"})
                    return
                self.order_id, self.order_update = o["orderId"], o["orderUpdateId"]
                with self.fw.lock:
                    self.fw.last_seq = -1          # sequence ids restart with each new order
                self.fw.set_goals(self._goals_from(o["nodes"], o.get("edges", [])))
            self.nodes = [n for n in o["nodes"]]
            self.edges = [e for e in o.get("edges", [])]

    def _handle_instant(self, a):
        t, aid = a.get("actionType"), a.get("actionId", "")
        if t == "startPause":
            self.fw.pause()
        elif t == "stopPause":
            self.fw.resume()
        elif t == "cancelOrder":
            self.fw.cancel()
            with self.lock:
                self.nodes, self.edges = [], []       # nothing left to drive
                with self.fw.lock:
                    self.fw.last_seq = -1
            with self.lock:
                for st in self.action_states.values():
                    if st["actionStatus"] in ("WAITING", "RUNNING"):
                        st["actionStatus"] = "FAILED"
        else:
            self.action_states[aid] = {"actionId": aid, "actionType": t, "actionStatus": "FAILED",
                                       "resultDescription": "unsupported"}
            return
        self.action_states[aid] = {"actionId": aid, "actionType": t, "actionStatus": "FINISHED"}

    # ------------------------------------------------------------- state
    def _state(self):
        fw = self.fw
        x, y, yaw = fw.pose()
        with fw.lock:
            if fw.action:
                st = self.action_states.get(fw.action["id"])
                if st:
                    st["actionStatus"] = "RUNNING"
            for aid, _, status in fw.finished_actions:
                if aid in self.action_states:
                    self.action_states[aid]["actionStatus"] = status
            fw.finished_actions.clear()
            remaining = {g["seq"] for g in fw.goals}
            last_seq = fw.last_seq
            last_node = fw.last_node or ""
        node_states = [{"nodeId": n["nodeId"], "sequenceId": n["sequenceId"], "released": n.get("released", True)}
                       for n in self.nodes if n["sequenceId"] > last_seq or n["sequenceId"] in remaining]
        edge_states = [{"edgeId": e["edgeId"], "sequenceId": e["sequenceId"], "released": e.get("released", True)}
                       for e in self.edges if e["sequenceId"] > last_seq]
        errors = list(self.errors) + [{"errorType": c, "errorLevel": "FATAL"} for c in fw.errors]
        return {
            **self._hdr(),
            "orderId": self.order_id, "orderUpdateId": self.order_update,
            "lastNodeId": last_node, "lastNodeSequenceId": last_seq,
            "nodeStates": node_states, "edgeStates": edge_states,
            "driving": abs(fw.v_cmd) > 0.01, "paused": fw.paused,
            "operatingMode": "AUTOMATIC", "newBaseRequest": False,
            "agvPosition": {"x": round(x, 3), "y": round(y, 3), "theta": round(yaw, 4),
                            "mapId": "warehouse", "positionInitialized": True},
            "velocity": {"vx": round(fw.measured_speed, 3), "vy": 0.0, "omega": 0.0},
            "batteryState": {"batteryCharge": round(fw.battery, 1), "charging": fw.charging},
            "safetyState": {"eStop": "NONE", "fieldViolation": fw.safety_stop},
            "errors": errors,
            "actionStates": list(self.action_states.values())[-20:],
            "loads": [],
        }

    def publish_state(self, force=False):
        st = self._state()
        sig = (st["lastNodeId"], st["driving"], st["paused"], st["safetyState"]["fieldViolation"],
               tuple(a["actionStatus"] for a in st["actionStates"]), len(st["errors"]))
        now = time.time()
        if force or sig != self._last_sig or now - self._last_pub >= 1.0 / self.state_hz:
            self.client.publish(f"{self.base}/state", json.dumps(st), qos=0)
            self._last_sig, self._last_pub = sig, now

    def tick(self):
        self.publish_state()
