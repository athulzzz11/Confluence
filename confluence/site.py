"""The active site map: traffic roadmap, named stations, chargers, speed zones,
task templates and static layout for the dashboard.

Sites live in confluence/sites/<name>.py. The active one is chosen by the
CONFLUENCE_SITE environment variable (default "warehouse"), or by calling
site.load(name) before anything else uses the map. Everything else in
Confluence reads the map through this module, so the same code runs any site.
Units: metres, radians, world frame (x east, y north).
"""
import heapq
import importlib
import math
import os

NAME = TITLE = None
BOUNDS = (0, 0, 1, 1)
LANES, STATIONS, CHARGERS, TASK_TEMPLATES, SPAWNS, PASSENGER_ROUTES = {}, {}, {}, {}, {}, {}
TASK_WORK, GUESTS = {}, {}
REST_AFTER_TASK = (0, 0)
SHAPES, SPEED_ZONES = [], []
NODES = {}   # id -> (x, y)
EDGES = {}   # id -> set(neighbour ids)
DEF = None   # the loaded site definition module (site-specific extras live there)

AVAILABLE = ("warehouse", "airport")


def _add_node(nid, x, y):
    NODES[nid] = (round(x, 3), round(y, 3))
    EDGES.setdefault(nid, set())


def _connect(a, b):
    EDGES[a].add(b)
    EDGES[b].add(a)


def node_at(x, y, tol=0.05):
    for nid, (nx, ny) in NODES.items():
        if abs(nx - x) < tol and abs(ny - y) < tol:
            return nid
    return None


def load(name=None):
    """(Re)load the active site. Safe to call again with another name."""
    global NAME, TITLE, BOUNDS, SHAPES, SPEED_ZONES, DEF, REST_AFTER_TASK
    name = (name or os.environ.get("CONFLUENCE_SITE") or "warehouse").strip().lower()
    if name not in AVAILABLE:
        raise ValueError(f"unknown site '{name}' (available: {', '.join(AVAILABLE)})")
    d = importlib.import_module(f"confluence.sites.{name}")
    DEF = d
    NAME, TITLE, BOUNDS = d.NAME, d.TITLE, d.BOUNDS
    SHAPES, SPEED_ZONES = list(d.SHAPES), list(d.SPEED_ZONES)
    for target, src in ((LANES, d.LANES), (TASK_TEMPLATES, d.TASK_TEMPLATES),
                        (SPAWNS, d.SPAWNS), (PASSENGER_ROUTES, d.PASSENGER_ROUTES)):
        target.clear()
        target.update(src)
    for target, src in ((TASK_WORK, getattr(d, 'TASK_WORK', {})), (GUESTS, getattr(d, 'GUESTS', {}))):
        target.clear()
        target.update(src)
    REST_AFTER_TASK = getattr(d, 'REST_AFTER_TASK', (0, 0))
    NODES.clear()
    EDGES.clear()
    CHARGERS.clear()
    STATIONS.clear()

    for lane, (y, xs) in d.LANES.items():
        prev = None
        for i, x in enumerate(xs):
            nid = f"{lane}{i}"
            _add_node(nid, x, y)
            if prev:
                _connect(prev, nid)
            prev = nid
    for x, lanes in d.CROSS:
        for a, b in zip(lanes, lanes[1:]):
            na, nb = node_at(x, d.LANES[a][0]), node_at(x, d.LANES[b][0])
            if not (na and nb):
                raise ValueError(f"site {name}: connector at x={x} needs nodes on lanes {a} and {b}")
            _connect(na, nb)
    for i, (lane, x, dock_y, _charger_y) in enumerate(d.CHARGERS):
        cid = f"CHG{i + 1}"
        _add_node(cid, x, dock_y)
        _connect(cid, node_at(x, d.LANES[lane][0]))
        CHARGERS[cid] = (x, dock_y)
    for sname, (lane, x) in d.STATIONS.items():
        nid = node_at(x, d.LANES[lane][0])
        if nid is None:
            raise ValueError(f"site {name}: station {sname} is not on a roadmap node")
        STATIONS[sname] = nid
    STATIONS.update({cid.lower(): cid for cid in CHARGERS})
    return d


def zone_limit(x, y):
    """Lowest speed limit at (x, y), or None outside all zones."""
    lim = None
    for _, x0, y0, x1, y1, v in SPEED_ZONES:
        if x0 <= x <= x1 and y0 <= y <= y1:
            lim = v if lim is None else min(lim, v)
    return lim


def dist(a, b):
    (ax, ay), (bx, by) = NODES[a], NODES[b]
    return math.hypot(ax - bx, ay - by)


def nearest_node(x, y, exclude_spurs=False):
    best, bd = None, 1e9
    for nid, (nx, ny) in NODES.items():
        if exclude_spurs and nid in CHARGERS:
            continue
        d = math.hypot(nx - x, ny - y)
        if d < bd:
            best, bd = nid, d
    return best, bd


def shortest_path(start, goal, blocked=(), penalty=None):
    """Dijkstra over the roadmap. `blocked` nodes are impassable (unless they
    are the start/goal); `penalty` maps node -> extra cost (used to route
    around congestion). Charger spurs are only entered if they are the goal."""
    blocked = set(blocked) - {start, goal}
    penalty = penalty or {}
    pq = [(0.0, start)]
    best = {start: 0.0}
    prev = {}
    while pq:
        d, n = heapq.heappop(pq)
        if n == goal:
            path = [n]
            while n in prev:
                n = prev[n]
                path.append(n)
            return path[::-1]
        if d > best.get(n, 1e18):
            continue
        for m in EDGES[n]:
            if m in blocked or (m in CHARGERS and m != goal):
                continue
            nd = d + dist(n, m) + penalty.get(m, 0.0)
            if nd < best.get(m, 1e18):
                best[m] = nd
                prev[m] = n
                heapq.heappush(pq, (nd, m))
    return None


def path_length(path):
    return sum(dist(a, b) for a, b in zip(path, path[1:]))


def collinear(a, b, c, tol=0.02):
    """True if travelling a->b->c needs no turn at b."""
    (ax, ay), (bx, by), (cx, cy) = NODES[a], NODES[b], NODES[c]
    h1 = math.atan2(by - ay, bx - ax)
    h2 = math.atan2(cy - by, cx - bx)
    return abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))) < tol


def as_dict():
    """Serializable site description for the dashboard."""
    return {
        "name": NAME, "title": TITLE,
        "bounds": BOUNDS,
        "nodes": {k: v for k, v in NODES.items()},
        "edges": sorted({tuple(sorted((a, b))) for a in EDGES for b in EDGES[a]}),
        "stations": STATIONS,
        "chargers": list(CHARGERS),
        "zones": SPEED_ZONES,
        "shapes": SHAPES,
    }


load()
