"""Procedurally builds the warehouse world SDF.

Run on every launch, so the environment is always created automatically:
    ros2 run confluence_sim make_world  [out.sdf] [--seed N]

Positions of racks, chargers, packing tables, lanes and speed zones come from
confluence/site.py (the facility map Confluence plans on), so the simulated
building and Confluence's map always agree.

Layout (top view, x right, y up; 20 m x 14 m):

    y= 7  ┌─────[dock door]────[dock door]────[dock door]─────┐
          │ pallets       ═════ conveyor ═════        pallets │
    y=4.8 │ ·  ·  ·  ·  ·  ·  ·  N2 dock lane ·  ·  ·  ·  ·  · │
    y=3.5 │ ·  ·  ·  ·  ·  ·  ·  N1 lane   ·  ·  ·  ·  ·  ·  · │
          │       [  racks  ]     |     [  racks  ]           │
    y= 0  │ ·  ·  ·  ·  ·  ·  ·  A main aisle ·  ·  ·  ·  ·  · │
          │       [  racks  ]     |     [  racks  ]           │
    y=-3.3│ ·  ·  ·  ·  ·  ·  ·  S1 lane   ·  ·  ·  ·  ·  ·  · │
    y=-5.15│ ·  ·  ·  ·  ·  ·  · S2 service lane ·  ·  ·  ·  · │
          │       C1   C2   C3      C4  [pack 1][pack 2]       │
    y=-7  └───────────────────────────────────────────────────┘
          vertical connectors at x = -8.1, 0 (cross-aisle), +8.1
"""
import math
import random
import sys

from confluence import site

W, D = 20.0, 14.0          # floor size (x, y)
WALL_H, WALL_T = 4.0, 0.2

# Every solid obstacle footprint (cx, cy, sx, sy) - used for route checking.
FOOTPRINTS = []

# ------------------------------------------------------------------ helpers
CARDBOARD = [(0.62, 0.46, 0.30), (0.70, 0.53, 0.36), (0.55, 0.40, 0.26), (0.76, 0.62, 0.44)]


def mat(rgb, a=1.0, emissive=None):
    r, g, b = rgb
    e = f"<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>" if emissive else ""
    return (f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular>{e}</material>")


def geom_box(sx, sy, sz):
    return f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"


def geom_cyl(r, l):
    return f"<geometry><cylinder><radius>{r:.3f}</radius><length>{l:.3f}</length></cylinder></geometry>"


class Model:
    """Collects visuals/collisions for one static model."""

    def __init__(self, name, x=0.0, y=0.0, yaw=0.0):
        self.name, self.x, self.y, self.yaw = name, x, y, yaw
        self.parts = []
        self.n = 0

    def _id(self):
        self.n += 1
        return f"p{self.n}"

    def box(self, x, y, z, sx, sy, sz, rgb, collide=True, roll=0.0, pitch=0.0, yaw=0.0, emissive=None):
        i = self._id()
        pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>"
        s = f'<visual name="v{i}">{pose}{geom_box(sx, sy, sz)}{mat(rgb, emissive=emissive)}</visual>'
        if collide:
            s += f'<collision name="c{i}">{pose}{geom_box(sx, sy, sz)}</collision>'
        self.parts.append(s)

    def cyl(self, x, y, z, r, l, rgb, collide=True, roll=0.0, pitch=0.0, emissive=None):
        i = self._id()
        pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} 0</pose>"
        s = f'<visual name="v{i}">{pose}{geom_cyl(r, l)}{mat(rgb, emissive=emissive)}</visual>'
        if collide:
            s += f'<collision name="c{i}">{pose}{geom_cyl(r, l)}</collision>'
        self.parts.append(s)

    def sphere(self, x, y, z, r, rgb):
        i = self._id()
        self.parts.append(f'<visual name="v{i}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
                          f'<geometry><sphere><radius>{r}</radius></sphere></geometry>{mat(rgb)}</visual>')

    def sdf(self):
        return (f'<model name="{self.name}"><static>true</static>'
                f'<pose>{self.x:.3f} {self.y:.3f} 0 0 0 {self.yaw:.4f}</pose>'
                f'<link name="link">{"".join(self.parts)}</link></model>')


def footprint(cx, cy, sx, sy):
    FOOTPRINTS.append((cx, cy, sx, sy))


# ------------------------------------------------------------------ building
def building():
    m = Model("building")
    hx, hy = W / 2, D / 2
    wall_rgb, stripe = (0.80, 0.82, 0.84), (0.25, 0.35, 0.55)
    for name, (x, y, sx, sy) in {
        "n": (0, hy, W, WALL_T), "s": (0, -hy, W, WALL_T),
        "e": (hx, 0, WALL_T, D), "w": (-hx, 0, WALL_T, D),
    }.items():
        m.box(x, y, WALL_H / 2, sx, sy, WALL_H, wall_rgb)
        # painted lower band, slightly proud of the wall face
        ox = -0.01 * math.copysign(1, x) if sx < 1 else 0
        oy = -0.01 * math.copysign(1, y) if sy < 1 else 0
        m.box(x + ox, y + oy, 0.6, sx, sy, 1.2, stripe, collide=False)
        footprint(x, y, sx, sy)

    # roll-up dock doors on the north wall, with black bumpers
    for dx in (-6.0, 0.0, 6.0):
        for k in range(12):
            m.box(dx, hy - 0.13, 0.15 + k * 0.26, 3.0, 0.04, 0.22, (0.42, 0.45, 0.48), collide=False)
        m.box(dx, hy - 0.13, 3.3, 3.3, 0.06, 0.12, (0.95, 0.75, 0.1), collide=False)   # header
        for bx in (-1.3, 1.3):
            m.box(dx + bx, hy - 0.2, 0.5, 0.25, 0.15, 0.4, (0.05, 0.05, 0.05))

    # roof truss beams + light fixtures (visual only, above robots)
    for x in (-7.5, -2.5, 2.5, 7.5):
        m.box(x, 0, WALL_H - 0.1, 0.15, D, 0.2, (0.55, 0.55, 0.58), collide=False)
    for x in (-5, 0, 5):
        for y in (-4, 0, 4):
            m.box(x, y, WALL_H - 0.35, 1.2, 0.25, 0.06, (1, 1, 0.95), collide=False, emissive=(1, 1, 0.9))

    # structural columns along east/west walls, with hazard bands
    for x in (-9.3, 9.3):
        for y in (-4.5, 0.0, 4.5):
            m.box(x, y, WALL_H / 2, 0.4, 0.4, WALL_H, (0.65, 0.65, 0.65))
            for k in range(4):
                c = (0.95, 0.8, 0.1) if k % 2 == 0 else (0.05, 0.05, 0.05)
                m.box(x, y, 0.1 + k * 0.2, 0.42, 0.42, 0.2, c, collide=False)
            footprint(x, y, 0.4, 0.4)

    # fire extinguishers + exit sign
    for x, y in ((9.75, -2.0), (-9.75, 2.0), (-9.75, -6.0)):
        m.cyl(x, y, 0.45, 0.08, 0.5, (0.85, 0.05, 0.05), collide=False)
        m.box(x, y, 1.2, 0.03, 0.3, 0.3, (0.9, 0.1, 0.1), collide=False)
    m.box(9.85, 6.2, 2.6, 0.05, 0.6, 0.25, (0.1, 0.8, 0.2), collide=False, emissive=(0, 0.6, 0.1))
    return m


def floor_and_markings():
    m = Model("floor")
    m.box(0, 0, -0.05, W, D, 0.1, (0.50, 0.51, 0.52))               # polished concrete
    yellow, white, green = (0.95, 0.80, 0.10), (0.95, 0.95, 0.95), (0.15, 0.65, 0.3)

    def line(x1, y1, x2, y2, rgb, w=0.08):
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        L = math.hypot(x2 - x1, y2 - y1)
        m.box(cx, cy, 0.002, L, w, 0.004, rgb, collide=False, yaw=math.atan2(y2 - y1, x2 - x1))

    # dashed guide lines along every roadmap edge (what Confluence plans on)
    for a, b in sorted({tuple(sorted((a, b))) for a in site.EDGES for b in site.EDGES[a]}):
        (x1, y1), (x2, y2) = site.NODES[a], site.NODES[b]
        if a in site.CHARGERS or b in site.CHARGERS:
            line(x1, y1, x2, y2, (0.1, 0.45, 0.95), 0.05)            # docking approach
            continue
        L = math.hypot(x2 - x1, y2 - y1)
        k, t = int(L / 0.5), 0
        while t < k:
            f0, f1 = t / k, min(1.0, (t + 0.6) / k)
            line(x1 + (x2 - x1) * f0, y1 + (y2 - y1) * f0, x1 + (x2 - x1) * f1, y1 + (y2 - y1) * f1, white, 0.05)
            t += 2
    # node markers (small yellow squares)
    for nid, (x, y) in site.NODES.items():
        if nid not in site.CHARGERS:
            m.box(x, y, 0.003, 0.16, 0.16, 0.004, yellow, collide=False)
    # speed-limited zones: yellow border
    for _, x0, y0, x1, y1, _v in site.SPEED_ZONES:
        x0, x1 = max(x0, -9.85), min(x1, 9.85)
        y0, y1 = max(y0, -6.85), min(y1, 6.85)
        for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)):
            line(ax, ay, bx, by, yellow, 0.1)
    # pedestrian walkway (green) along the west wall
    line(-9.7, -6.5, -9.7, 6.5, green, 0.15)
    # hatched keep-clear zone in front of dock conveyor
    for k in range(13):
        x = -3 + k * 0.5
        line(x, 5.55, x + 0.35, 5.8, yellow, 0.06)
    # charging bay outlines (blue)
    for x in site.DEF.CHARGER_X:
        for dx in (-0.5, 0.5):
            line(x + dx, -6.9, x + dx, -5.75, (0.1, 0.45, 0.95), 0.06)
    return m


def rack_block(name, x0, x1, y, rng):
    """Selective pallet racking: blue uprights, orange beams, decks, mixed stock."""
    m = Model(name)
    depth, height = 1.0, 3.0
    levels = [0.12, 1.0, 1.85, 2.7]
    bays = int(round((x1 - x0) / 2.5))
    bay_w = (x1 - x0) / bays
    blue, orange, deck = (0.10, 0.25, 0.60), (0.95, 0.45, 0.05), (0.55, 0.57, 0.60)
    for b in range(bays + 1):
        ux = x0 + b * bay_w
        for uy in (y - depth / 2 + 0.04, y + depth / 2 - 0.04):
            m.box(ux, uy, height / 2, 0.08, 0.08, height, blue, collide=False)
        for z in (0.6, 1.5, 2.4):   # cross bracing
            m.box(ux, y, z, 0.04, depth - 0.1, 0.04, blue, collide=False, roll=0.6)
    for z in levels[1:]:
        for uy in (y - depth / 2 + 0.04, y + depth / 2 - 0.04):
            m.box((x0 + x1) / 2, uy, z, x1 - x0, 0.06, 0.1, orange, collide=False)
        m.box((x0 + x1) / 2, y, z + 0.055, x1 - x0, depth - 0.1, 0.01, deck, collide=False)
    # stock on each bay/level
    for b in range(bays):
        bx0 = x0 + b * bay_w + 0.15
        for li, z in enumerate(levels):
            base = z + (0.06 if li else 0.0)
            if li == 0 or rng.random() < 0.5:     # a pallet with boxes
                px = bx0 + bay_w / 2 - 0.15
                m.box(px, y, base + 0.07, 1.2, 0.8, 0.14, (0.6, 0.45, 0.25), collide=False)
                stack_h = 0.0
                for _ in range(rng.randint(1, 2)):
                    h = rng.uniform(0.25, 0.4)
                    m.box(px, y, base + 0.14 + stack_h + h / 2, 1.1, 0.75, h, rng.choice(CARDBOARD), collide=False)
                    stack_h += h
            else:                                 # loose cartons
                cx = bx0
                while cx < bx0 + bay_w - 0.5:
                    sx, sz = rng.uniform(0.3, 0.55), rng.uniform(0.2, 0.45)
                    if rng.random() < 0.8:
                        m.box(cx + sx / 2, y + rng.uniform(-0.1, 0.1), base + sz / 2, sx, rng.uniform(0.4, 0.7), sz,
                              rng.choice(CARDBOARD), collide=False)
                    cx += sx + 0.05
    # one simple collision volume for the whole block (fast + robust)
    m.parts.append(f'<collision name="block"><pose>{(x0 + x1) / 2} {y} {height / 2} 0 0 0</pose>'
                   f'{geom_box(x1 - x0, depth, height)}</collision>')
    # end-of-aisle guards (yellow posts)
    for gx in (x0 - 0.15, x1 + 0.15):
        for gy in (y - 0.4, y + 0.4):
            m.cyl(gx, gy, 0.4, 0.06, 0.8, (0.95, 0.8, 0.1))
    footprint((x0 + x1) / 2, y, x1 - x0 + 0.42, depth)
    return m


def dock_area(rng):
    m = Model("dock_area")
    # conveyor with legs, rollers, side rails and parcels
    cx0, cx1, cy, ch = -3.0, 3.0, 6.1, 0.8
    m.box(0, cy, ch - 0.05, cx1 - cx0, 0.7, 0.1, (0.3, 0.3, 0.32), collide=False)
    for x in (cx0 + 0.2, -1, 1, cx1 - 0.2):
        for y in (cy - 0.3, cy + 0.3):
            m.box(x, y, (ch - 0.1) / 2, 0.06, 0.06, ch - 0.1, (0.3, 0.3, 0.32), collide=False)
    x = cx0 + 0.1
    while x < cx1:
        m.cyl(x, cy, ch + 0.02, 0.03, 0.6, (0.75, 0.75, 0.78), collide=False, roll=math.pi / 2)
        x += 0.12
    for y in (cy - 0.33, cy + 0.33):
        m.box(0, y, ch + 0.1, cx1 - cx0, 0.03, 0.12, (0.95, 0.75, 0.1), collide=False)
    for px in (-2.2, -0.9, 0.4, 1.9):
        s = rng.uniform(0.25, 0.4)
        m.box(px, cy, ch + 0.05 + s / 2, s, s * 1.2, s, rng.choice(CARDBOARD), collide=False)
    m.parts.append(f'<collision name="conv"><pose>0 {cy} {ch / 2} 0 0 0</pose>{geom_box(cx1 - cx0, 0.7, ch)}</collision>')
    footprint(0, cy, cx1 - cx0, 0.7)
    # staged pallets near the doors
    for (x, y) in ((-7.0, 6.2), (-5.5, 6.2), (5.5, 6.2), (7.0, 6.2)):
        m.box(x, y, 0.07, 1.2, 0.8, 0.14, (0.6, 0.45, 0.25))
        h = 0.14
        for _ in range(rng.randint(1, 3)):
            sh = rng.uniform(0.3, 0.45)
            m.box(x, y, h + sh / 2, 1.15, 0.78, sh, rng.choice(CARDBOARD))
            h += sh
        m.box(x, y, h + 0.01, 1.16, 0.79, 0.02, (0.85, 0.9, 0.95), collide=False)   # stretch-wrap top
        footprint(x, y, 1.2, 0.8)
    # pallet jack
    m.box(3.9, 6.55, 0.05, 1.15, 0.55, 0.08, (0.85, 0.15, 0.1))          # parked against the wall
    m.box(3.3, 6.55, 0.5, 0.04, 0.04, 0.9, (0.1, 0.1, 0.1), collide=False, roll=0.0, pitch=0.3)
    footprint(3.9, 6.55, 1.15, 0.55)
    # traffic cones
    for x in (-3.3, 3.3):
        m.cyl(x, 5.75, 0.2, 0.12, 0.4, (1.0, 0.4, 0.0))
        m.cyl(x, 5.75, 0.25, 0.122, 0.06, (1, 1, 1), collide=False)
        footprint(x, 5.75, 0.25, 0.25)
    return m


def charging_and_packing():
    m = Model("service_area")
    # charging stations along the south wall
    for x in site.DEF.CHARGER_X:
        m.box(x, -6.7, 0.35, 0.6, 0.3, 0.7, (0.2, 0.2, 0.22))
        m.box(x, -6.54, 0.55, 0.3, 0.02, 0.12, (0.1, 0.9, 0.3), collide=False, emissive=(0.0, 0.7, 0.2))
        m.box(x, -6.45, 0.12, 0.4, 0.2, 0.06, (0.6, 0.5, 0.1), collide=False)   # contact plate
        footprint(x, -6.7, 0.6, 0.3)
    # packing stations: table, monitor, parcels, a worker
    for i, x in enumerate(site.DEF.PACK_TABLE_X):
        m.box(x, -6.45, 0.85, 1.4, 0.6, 0.05, (0.75, 0.72, 0.65))
        for lx in (-0.65, 0.65):
            for ly in (-0.25, 0.25):
                m.box(x + lx, -6.45 + ly, 0.42, 0.05, 0.05, 0.84, (0.3, 0.3, 0.3), collide=False)
        m.box(x - 0.4, -6.6, 1.1, 0.4, 0.03, 0.28, (0.05, 0.05, 0.08), collide=False)
        m.box(x + 0.3, -6.4, 0.98, 0.35, 0.3, 0.22, CARDBOARD[i % 4], collide=False)
        m.parts.append(f'<collision name="tbl{i}"><pose>{x} -6.45 0.45 0 0 0</pose>{geom_box(1.4, 0.6, 0.9)}</collision>')
        footprint(x, -6.45, 1.4, 0.6)
        # worker standing at the end of the table (hi-vis vest)
        wx = x - 0.95
        m.cyl(wx, -6.45, 0.45, 0.12, 0.9, (0.15, 0.2, 0.35))           # legs
        m.cyl(wx, -6.45, 1.2, 0.18, 0.6, (1.0, 0.55, 0.05))            # vest
        m.sphere(wx, -6.45, 1.65, 0.12, (0.85, 0.65, 0.5))             # head
        m.cyl(wx, -6.45, 1.79, 0.13, 0.06, (0.95, 0.85, 0.1), collide=False)  # hard hat
        footprint(wx, -6.45, 0.36, 0.36)
    return m


# ------------------------------------------------------------------ world
def point_lights(points, z=3.5, rng=12):
    return "".join(
        f'<light type="point" name="lamp_{i}"><pose>{x} {y} {z} 0 0 0</pose>'
        f'<diffuse>0.6 0.6 0.55 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
        f'<attenuation><range>{rng}</range><constant>0.5</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation>'
        f'<cast_shadows>false</cast_shadows></light>'
        for i, (x, y) in enumerate(points))


def world_sdf(name, models, lights="", ambient=0.55, background=(0.7, 0.75, 0.8)):
    bg = " ".join(str(c) for c in background)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>{ambient} {ambient} {ambient} 1</ambient>
      <background>{bg} 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    {lights}
    {"".join(mm.sdf() for mm in models)}
  </world>
</sdf>
"""


def build_warehouse(seed=7):
    FOOTPRINTS.clear()
    rng = random.Random(seed)
    models = [building(), floor_and_markings()]
    for i, (x0, x1, y) in enumerate(site.DEF.RACK_BLOCKS):
        models.append(rack_block(f"rack_{i}", x0, x1, y, rng))
    models += [dock_area(rng), charging_and_packing()]
    return world_sdf(site.NAME, models, point_lights(((-5, -3.5), (5, -3.5), (-5, 3.5), (5, 3.5))))


def build_world(seed=7):
    """Generate the world for the active site (confluence.site)."""
    if site.NAME == "airport":
        from confluence_sim.world_airport import build_airport
        return build_airport(seed)
    return build_warehouse(seed)


def main(argv=None):
    """make_world [out.sdf] [--seed N] [--site warehouse|airport]"""
    argv = list(sys.argv[1:] if argv is None else argv)
    seed = 7
    for flag in ("--seed", "--site"):
        if flag in argv:
            i = argv.index(flag)
            if flag == "--seed":
                seed = int(argv[i + 1])
            else:
                site.load(argv[i + 1])
            del argv[i:i + 2]
    out = argv[0] if argv else f"{site.NAME}.sdf"
    from confluence_sim import world_gen as pkg   # same module object the site builders use
    with open(out, "w") as f:
        f.write(pkg.build_world(seed))
    print(f"wrote {out} ({len(pkg.FOOTPRINTS)} obstacles, site {site.NAME}, seed {seed})")


if __name__ == "__main__":
    main()
