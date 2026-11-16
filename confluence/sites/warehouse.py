"""Site definition: 20 m x 14 m warehouse (racks, dock, chargers, packing)."""
NAME = "warehouse"
TITLE = "Warehouse"
BOUNDS = (-10.0, -7.0, 10.0, 7.0)

_X = [-8.1, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.1]
LANES = {            # lane id -> (y, node x positions)
    "N2": (4.8, _X),     # dock lane
    "N1": (3.5, _X),     # north rack face
    "A": (0.0, _X),      # main aisle between racks
    "S1": (-3.3, _X),    # south rack face
    "S2": (-5.15, _X),   # service lane (chargers, packing)
}
# vertical links between lanes, only where nothing stands in the way (racks sit between N1/A/S1)
CROSS = ([(x, ["N2", "N1", "A", "S1", "S2"]) for x in (-8.1, 0.0, 8.1)]                 # full cross-aisles
         + [(x, ["N2", "N1"]) for x in (-6.0, -4.0, -2.0, 2.0, 4.0, 6.0)]                # dock side
         + [(x, ["S1", "S2"]) for x in (-6.0, -4.0, -2.0, 2.0, 4.0, 6.0)])               # service side

CHARGER_X = [-6.0, -4.0, -2.0, 2.0]
CHARGER_Y = -6.7                    # charger body (against south wall)
DOCK_Y = -6.08                      # robot centre when docked
CHARGERS = [("S2", x, DOCK_Y, CHARGER_Y) for x in CHARGER_X]   # (lane, x, dock y, charger y)

RACK_BLOCKS = [(-6.0, -1.0, 1.5), (1.0, 6.0, 1.5), (-6.0, -1.0, -1.5), (1.0, 6.0, -1.5)]  # x0, x1, y
PACK_TABLE_X = [4.0, 6.0]
PACK_TABLE_Y = -6.45
DOOR_X = [-6.0, 0.0, 6.0]

SHAPES = (
    [("rack", (x0 + x1) / 2, y, x1 - x0, 1.0, "Racks") for x0, x1, y in RACK_BLOCKS]
    + [("conveyor", 0.0, 6.1, 6.0, 0.7, "Conveyor")]
    + [("pallet", x, 6.2, 1.2, 0.8, "") for x in (-7.0, -5.5, 5.5, 7.0)]
    + [("charger", x, CHARGER_Y, 0.6, 0.3, f"C{i + 1}") for i, x in enumerate(CHARGER_X)]
    + [("table", x, PACK_TABLE_Y, 1.4, 0.6, f"Pack {i + 1}") for i, x in enumerate(PACK_TABLE_X)]
    + [("door", x, 6.95, 3.0, 0.1, "") for x in DOOR_X]
)

SPEED_ZONES = [
    ("packing_area", 1.0, -7.0, 9.0, -4.4, 0.25),
    ("dock_area", -9.0, 4.2, 9.0, 7.0, 0.30),
]

STATIONS = {          # name -> (lane, x)
    "dock_west": ("N2", -6.0), "dock_centre": ("N2", 0.0), "dock_east": ("N2", 6.0),
    "rack_aisle_w": ("A", -4.0), "rack_aisle_e": ("A", 4.0),
    "rack_north_w": ("N1", -4.0), "rack_north_e": ("N1", 4.0),
    "rack_south_w": ("S1", -4.0), "rack_south_e": ("S1", 4.0),
    "pack_1": ("S2", 4.0), "pack_2": ("S2", 6.0),
}

# kind -> (capability, pickup choices, dropoff choices)
TASK_TEMPLATES = {
    "move_pallet": ("pallet", ["dock_west", "dock_east"], ["rack_aisle_w", "rack_aisle_e"]),
    "pick_tote":   ("tote", ["rack_north_w", "rack_north_e", "rack_south_w", "rack_south_e"], ["pack_1", "pack_2"]),
    "deliver_bin": ("bin", ["dock_centre", "dock_west"], ["rack_north_w", "rack_north_e", "rack_south_e"]),
    "tow_cart":    ("tow", ["dock_centre", "dock_east"], ["pack_1", "pack_2"]),
}

TASK_WORK = {
    "move_pallet": {"pick": ("lifting the pallet", 10), "drop": ("placing the pallet", 10),
                    "speed": 0.25, "work": "carrying a pallet"},
    "pick_tote":   {"pick": ("waiting for the picker", 9), "drop": ("handing over the tote", 6),
                    "work": "carrying a tote"},
    "deliver_bin": {"pick": ("loading the bin", 7), "drop": ("unloading the bin", 7),
                    "work": "carrying a bin"},
    "tow_cart":    {"pick": ("coupling the cart", 10), "drop": ("uncoupling the cart", 8),
                    "speed": 0.3, "work": "towing a cart"},
}
REST_AFTER_TASK = (5, 10)

SPAWNS = {"r1": (-6.0, 0.0), "r2": (6.0, 4.8), "r3": (0.0, -3.3), "r4": (8.1, -5.15)}
PASSENGER_ROUTES = {}
GUESTS = {}    # no pedestrians walking here
