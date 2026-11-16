"""Site definition: airport terminal (indoor), 24 m x 16 m.

    y= 8  ═══ glass facade ══[gate 1]══════[gate 2]══════[gate 3]═══  (aircraft outside)
          [ seating ]   podium  [ seating ]  podium  [ seating ]   podium [ seating ]
    y=4.3  · · · · · · · · · ·  G  gate lane  · · · · · · · · · ·
          [   cafe   ]  [info]        [duty free]   [ news & books ]
    y=1.2  · · · · · · · · · ·  CN concourse north · · · · · · · ·   baggage hall door ▶
                planter                 planter
    y=-1.4 · · · · · · · · · ·  CS concourse south · · · · · · · ·
     trolleys  column                              column    trolleys
    y=-3.8 · · · · · · · · · ·  K  check-in lane  · · · · · · · ·
           C1   C2   C3     C4  |  queue barriers / passengers in line
          ─── charging room ────|  [counter][counter][counter][counter][counter] [oversize]
    y=-8  ══════════════════════╧═══════ baggage belt ═══════════════════════
    vertical connectors at x = -10.5, 0, +10.5
"""
NAME = "airport"
TITLE = "Airport terminal"
BOUNDS = (-12.0, -8.0, 12.0, 8.0)

_X = [-10.5, -9.0, -7.5, -6.0, -3.0, 0.0, 3.0, 6.0, 7.5, 9.0, 10.5]
LANES = {
    "G": (4.3, _X),      # gate lane, in front of the seating
    "CN": (1.2, _X),     # concourse north, in front of shops
    "CS": (-1.4, _X),    # concourse south
    "K": (-3.8, _X),     # check-in lane
}
# vertical links between lanes, only where nothing stands in the way
CROSS = ([(x, ["G", "CN", "CS", "K"]) for x in (-10.5, 0.0, 10.5)]                # full cross-aisles
         + [(x, ["CN", "CS"]) for x in (-7.5, -3.0, 3.0, 7.5)]                  # across the concourse
         + [(x, ["CS", "K"]) for x in (-9.0, -6.0, -3.0, 3.0, 6.0, 9.0)])        # to the check-in lane

CHARGER_X = [-9.0, -7.5, -6.0, -3.0]
CHARGER_Y = -5.4
DOCK_Y = -4.8
CHARGERS = [("K", x, DOCK_Y, CHARGER_Y) for x in CHARGER_X]

GATE_X = [-6.0, 0.0, 6.0]
SEAT_BLOCKS = [(-11.3, -7.3), (-4.7, -1.3), (1.3, 4.7), (7.3, 11.3)]    # x ranges, y 5.3..6.3
SEAT_Y0, SEAT_Y1 = 5.3, 6.3
KIOSKS = [  # (kind, x0, x1, y0, y1, label)
    ("cafe", -9.0, -4.6, 2.2, 3.3, "Cafe"),
    ("desk", -3.8, -2.2, 2.7, 3.3, "Info"),
    ("shop", 1.2, 4.8, 2.2, 3.3, "Duty Free"),
    ("shop", 6.2, 9.2, 2.2, 3.3, "News & Books"),
]
COUNTER_X = [1.6, 3.4, 5.2, 7.0, 8.8]       # check-in counters, 1.4 m wide
COUNTER_Y = -6.9
QUEUE = (1.0, 10.0, -6.15, -4.75)           # x0, x1, y0, y1 (barriers + waiting passengers)
OVERSIZE = (10.6, 11.8, -6.0, -4.9)         # oversize baggage drop counter
PARTITION_Y = -5.9                          # charging room back wall (x -11.9 .. -1.8)
PLANTERS = [(-8.6, -0.1), (-6.4, -0.1), (6.4, -0.1), (8.6, -0.1)]
COLUMNS = [(-7.5, -2.6), (7.5, -2.6)]
TROLLEY_ROWS = [(-11.75, -3.2, -2.0), (11.75, -3.2, -2.0)]   # x, y0, y1 along the side walls

SHAPES = (
    [("seats", (a + b) / 2, (SEAT_Y0 + SEAT_Y1) / 2, b - a, SEAT_Y1 - SEAT_Y0, "Seating") for a, b in SEAT_BLOCKS]
    + [("podium", x, 7.0, 1.2, 0.6, f"Gate {i + 1}") for i, x in enumerate(GATE_X)]
    + [("door", x, 7.95, 2.0, 0.1, "") for x in GATE_X]
    + [(k, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, lbl) for k, x0, x1, y0, y1, lbl in KIOSKS]
    + [("planter", x, y, 0.7, 0.7, "") for x, y in PLANTERS]
    + [("column", x, y, 0.5, 0.5, "") for x, y in COLUMNS]
    + [("trolleys", x, (a + b) / 2, 0.3, b - a, "") for x, a, b in TROLLEY_ROWS]
    + [("queue", (QUEUE[0] + QUEUE[1]) / 2, (QUEUE[2] + QUEUE[3]) / 2, QUEUE[1] - QUEUE[0], QUEUE[3] - QUEUE[2], "Check-in queue")]
    + [("counter", x, COUNTER_Y, 1.4, 0.7, "") for x in COUNTER_X]
    + [("counter", (OVERSIZE[0] + OVERSIZE[1]) / 2, (OVERSIZE[2] + OVERSIZE[3]) / 2,
        OVERSIZE[1] - OVERSIZE[0], OVERSIZE[3] - OVERSIZE[2], "Oversize")]
    + [("belt", 5.0, -7.65, 10.6, 0.5, "Baggage belt")]
    + [("partition", -6.85, PARTITION_Y, 10.1, 0.1, "")]
    + [("charger", x, CHARGER_Y, 0.6, 0.3, f"C{i + 1}") for i, x in enumerate(CHARGER_X)]
    + [("door", 11.95, 1.2, 0.1, 2.0, "")]
)

SPEED_ZONES = [
    ("checkin_hall", -1.5, -8.0, 12.0, -2.6, 0.25),
    ("gate_seating", -12.0, 3.7, 12.0, 8.0, 0.30),
    ("concourse", -12.0, -2.6, 12.0, 3.7, 0.45),
]

STATIONS = {
    "gate_1": ("G", -6.0), "gate_2": ("G", 0.0), "gate_3": ("G", 6.0),
    "cafe": ("CN", -6.0), "info_desk": ("CN", -3.0), "duty_free": ("CN", 3.0),
    "baggage_hall_door": ("CN", 10.5),
    "trolley_bay_w": ("CS", -10.5), "trolley_bay_e": ("CS", 10.5), "security_post": ("CS", 9.0),
    "trolley_park": ("K", 0.0), "checkin_hall": ("K", 6.0), "oversize_drop": ("K", 9.0),
}

TASK_TEMPLATES = {
    "guide_passenger":   ("guide", ["info_desk"], ["gate_1", "gate_2", "gate_3"]),
    "clean_area":        ("clean", ["gate_1", "gate_2", "gate_3", "cafe"], ["checkin_hall", "gate_2", "cafe"]),
    "security_patrol":   ("patrol", ["security_post"], ["gate_1", "gate_3", "cafe", "checkin_hall"]),
    "collect_trolleys":  ("tow", ["trolley_bay_w", "trolley_bay_e"], ["trolley_park"]),
    "move_oversize_bag": ("baggage", ["oversize_drop"], ["baggage_hall_door"]),
}

# What each job physically involves: (label, seconds) at pickup and dropoff, an optional
# working speed while carrying it out, and whether a passenger walks along (escort).
TASK_WORK = {
    "guide_passenger":   {"pick": ("greeting the passenger", 8), "drop": ("showing the gate", 7),
                          "speed": 0.4, "work": "escorting the passenger", "escort": True},
    "clean_area":        {"pick": ("lowering the brushes", 5), "drop": ("emptying the tank", 9),
                          "speed": 0.2, "work": "scrubbing the floor"},
    "security_patrol":   {"pick": ("checking in at the post", 4), "drop": ("inspecting the area", 12),
                          "work": "on patrol"},
    "collect_trolleys":  {"pick": ("coupling the trolleys", 10), "drop": ("uncoupling the trolleys", 8),
                          "speed": 0.3, "work": "towing trolleys"},
    "move_oversize_bag": {"pick": ("loading the bag", 12), "drop": ("unloading the bag", 10),
                          "work": "carrying the bag"},
}
REST_AFTER_TASK = (6, 12)      # seconds a robot stays put after finishing a job

SPAWNS = {"r1": (-3.0, 1.2), "r2": (6.0, 4.3), "r3": (0.0, -1.4), "r4": (9.0, -3.8)}

# Robot body styling for this site (same chassis sizes as the warehouse fleet)
ROBOT_KINDS = {"r1": "guide", "r2": "scrubber", "r3": "tugger", "r4": "baggage_agv"}

# Passengers walking through the terminal (not robots - Confluence doesn't know
# about them; only each robot's own lidar does). name -> list of waypoints.
PASSENGER_ROUTES = {
    "p1": [(-4.5, -2.9), (-4.5, 1.85)],     # check-in -> cafe, crossing both concourse lanes
    "p2": [(4.5, 1.85), (4.5, -2.9)],       # duty free -> check-in
    "p3": [(-3.6, -0.1), (3.6, -0.1)],      # strolling along the concourse, crossing the centre aisle
    "p4": [(9.7, -2.9), (9.7, 1.9)],        # check-in -> baggage hall side
}

# A passenger who asks the guide robot for help: waits by the info desk, walks behind the
# robot to the gate, stays there a while, then walks back (as the next passenger).
GUESTS = {
    "guest": {"robot": "r1", "wait": (-1.6, 1.95), "stay_offset": (0.0, 0.6), "stay_s": 25,
              "return_via": [(-1.5, 4.9), (-1.5, 1.95)]},
}
