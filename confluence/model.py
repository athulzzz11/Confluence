"""Unified schema. Every adapter converts its vendor's telemetry into a
RobotState, so the rest of Confluence never sees vendor-specific units,
frames, state names or update rates."""
import itertools
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional

# Normalized operating states
IDLE, MOVING, PAUSED, ERROR, CHARGING, OFFLINE, ACTING = (
    "IDLE", "MOVING", "PAUSED", "ERROR", "CHARGING", "OFFLINE", "ACTING")


@dataclass
class RobotState:
    robot_id: str
    vendor: str = ""
    model: str = ""
    protocol: str = ""
    x: float = 0.0              # m, world frame
    y: float = 0.0
    yaw: float = 0.0            # rad, 0 = +x (east), CCW positive
    speed: float = 0.0          # m/s (measured or reported)
    battery: float = 100.0      # % 0..100
    state: str = OFFLINE        # one of the normalized states above
    charging: bool = False
    safety_stop: bool = False   # robot's own safety system is holding it
    errors: List[str] = field(default_factory=list)
    action_done: bool = True    # last native action finished (VDA 5050)
    online: bool = False
    last_update: float = 0.0    # time.time() of last telemetry
    telemetry_hz: float = 0.0   # measured incoming update rate
    extra: dict = field(default_factory=dict)  # vendor-specific extras worth showing (e.g. escort status)
    raw: dict = field(default_factory=dict)  # last vendor message, for audit/debug

    def to_dict(self):
        d = asdict(self)
        d.pop("raw", None)
        return d


_task_ids = itertools.count(1)


@dataclass
class Task:
    kind: str                   # e.g. move_pallet, pick_tote
    capability: str             # robot must have this capability
    pickup: str                 # station name
    dropoff: str
    priority: int = 1
    task_id: str = field(default_factory=lambda: f"T{next(_task_ids):04d}")
    status: str = "QUEUED"      # QUEUED, ASSIGNED, TO_PICKUP, PICKING, TO_DROPOFF, DROPPING, DONE, FAILED
    robot_id: Optional[str] = None
    created: float = field(default_factory=time.time)
    assigned_at: Optional[float] = None
    finished_at: Optional[float] = None
    source: str = "auto"        # auto | operator | ros

    def to_dict(self):
        return asdict(self)


# Task templates (kind -> capability, pickups, dropoffs) are defined per site: confluence/sites/<name>.py
