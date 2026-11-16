"""Procedural airport terminal world (indoor departures level).

Everything is placed from confluence/sites/airport.py, so the building and
Confluence's map always agree. Generated on every launch with site:=airport.
"""
import math
import random

from confluence import site
from confluence_sim.world_gen import FOOTPRINTS, Model, footprint, point_lights, world_sdf

WALL_H = 5.0
GLASS = (0.55, 0.75, 0.9)
STEEL = (0.62, 0.64, 0.68)
DARK = (0.12, 0.13, 0.15)
WHITE = (0.94, 0.95, 0.96)
SIGN_BG = (0.08, 0.1, 0.14)
AMBER = (1.0, 0.72, 0.15)
CLOTHES = [(0.75, 0.15, 0.2), (0.15, 0.35, 0.7), (0.25, 0.55, 0.3), (0.5, 0.3, 0.6), (0.85, 0.6, 0.15),
           (0.2, 0.2, 0.22), (0.9, 0.9, 0.88), (0.4, 0.25, 0.15)]
LUGGAGE = [(0.15, 0.15, 0.17), (0.55, 0.1, 0.12), (0.12, 0.3, 0.55), (0.6, 0.6, 0.62), (0.75, 0.55, 0.2)]


def person(m, x, y, rng, yaw=0.0, sitting=False, bag=True, vest=None):
    """Static person (visual only, robots never path through these areas)."""
    c = vest or rng.choice(CLOTHES)
    skin = rng.choice([(0.85, 0.68, 0.55), (0.7, 0.52, 0.4), (0.5, 0.36, 0.27), (0.92, 0.78, 0.66)])
    cs, sn = math.cos(yaw), math.sin(yaw)
    if sitting:
        m.box(x + 0.15 * cs, y + 0.15 * sn, 0.5, 0.4, 0.3, 0.12, (0.2, 0.22, 0.3), collide=False, yaw=yaw)  # thighs
        m.box(x + 0.32 * cs, y + 0.32 * sn, 0.24, 0.12, 0.28, 0.46, (0.2, 0.22, 0.3), collide=False, yaw=yaw)  # shins
        m.box(x, y, 0.82, 0.22, 0.38, 0.55, c, collide=False, yaw=yaw)
        m.sphere(x, y, 1.22, 0.1, skin)
    else:
        for side in (-0.08, 0.08):
            m.cyl(x - sn * side, y + cs * side, 0.43, 0.06, 0.86, (0.15, 0.15, 0.2), collide=False)
        m.box(x, y, 1.15, 0.22, 0.4, 0.6, c, collide=False, yaw=yaw)
        m.sphere(x, y, 1.6, 0.11, skin)
        if bag:
            bx, by = x - 0.3 * cs + 0.2 * sn, y - 0.3 * sn - 0.2 * cs
            m.box(bx, by, 0.27, 0.22, 0.38, 0.5, rng.choice(LUGGAGE), collide=False, yaw=yaw)
            m.box(bx, by, 0.7, 0.02, 0.25, 0.36, (0.6, 0.6, 0.6), collide=False, yaw=yaw)


def sign(m, x, y, z, w, h, yaw=0.0, color=AMBER):
    """Back-lit overhead sign (dark panel with a glowing band)."""
    m.box(x, y, z, w, 0.08, h, SIGN_BG, collide=False, yaw=yaw)
    dx, dy = -math.sin(yaw) * 0.045, math.cos(yaw) * 0.045
    for s in (1, -1):
        m.box(x + s * dx, y + s * dy, z, w * 0.85, 0.01, h * 0.35, color, collide=False, yaw=yaw, emissive=color)


# ------------------------------------------------------------------ shell + outside
def building():
    m = Model("terminal_building")
    x0, y0, x1, y1 = site.BOUNDS
    W, D = x1 - x0, y1 - y0
    # polished terrazzo floor with a darker inlay strip along the concourse
    m.box(0, 0, -0.05, W, D, 0.1, (0.78, 0.76, 0.72))
    m.box(0, -0.1, 0.002, W - 0.4, 2.0, 0.004, (0.68, 0.66, 0.62), collide=False)
    # solid walls (south, east, west) with a stone base band
    for (x, y, sx, sy) in ((0, y0, W, 0.25), (x1, 0, 0.25, D), (x0, 0, 0.25, D)):
        m.box(x, y, WALL_H / 2, sx, sy, WALL_H, (0.86, 0.86, 0.84))
        m.box(x, y, 0.5, sx + 0.02, sy + 0.02, 1.0, (0.45, 0.43, 0.42), collide=False)
        footprint(x, y, sx, sy)
    # north glass facade: mullions + glass panes, collision as one wall
    m.parts.append(f'<collision name="facade"><pose>0 {y1} {WALL_H / 2} 0 0 0</pose>'
                   f'<geometry><box><size>{W} 0.2 {WALL_H}</size></box></geometry></collision>')
    footprint(0, y1, W, 0.2)
    x = x0
    while x < x1 - 0.01:
        m.box(x, y1, WALL_H / 2, 0.1, 0.22, WALL_H, STEEL, collide=False)
        x += 2.0
    for z in (0.05, 2.6, WALL_H - 0.05):
        m.box(0, y1, z, W, 0.22, 0.1, STEEL, collide=False)
    pane = Model("facade_glass")    # separate model so it can be transparent
    pane.parts.append(f'<visual name="glass"><pose>0 {y1} {WALL_H / 2} 0 0 0</pose>'
                      f'<geometry><box><size>{W} 0.04 {WALL_H}</size></box></geometry>'
                      f'<material><ambient>{GLASS[0]} {GLASS[1]} {GLASS[2]} 0.25</ambient>'
                      f'<diffuse>{GLASS[0]} {GLASS[1]} {GLASS[2]} 0.25</diffuse></material>'
                      f'<transparency>0.75</transparency></visual>')
    # jet-bridge doors at each gate
    for gx in site.DEF.GATE_X:
        m.box(gx, y1 - 0.12, 1.25, 2.0, 0.06, 2.5, (0.3, 0.32, 0.36), collide=False)
        m.box(gx, y1 - 0.15, 2.6, 2.2, 0.08, 0.2, STEEL, collide=False)
    # baggage-hall door (east wall) + check-in entrance doors (west wall)
    m.box(x1 - 0.14, 1.2, 1.3, 0.06, 2.0, 2.6, (0.35, 0.36, 0.4), collide=False)
    sign(m, x1 - 0.2, 1.2, 3.0, 1.6, 0.35, yaw=math.pi / 2, color=(0.95, 0.95, 0.95))
    for dy in (-3.0, 3.0):
        m.box(x0 + 0.14, dy, 1.3, 0.06, 2.4, 2.6, GLASS, collide=False)
    # roof structure, light rafts
    for x in range(-10, 11, 4):
        m.box(x, 0, WALL_H - 0.15, 0.25, D, 0.3, (0.8, 0.8, 0.82), collide=False)
    for x in (-8, -3, 3, 8):
        for y in (-5, -1.4, 2.5, 5.8):
            m.box(x, y, WALL_H - 0.4, 2.2, 0.5, 0.06, (1, 1, 0.97), collide=False, emissive=(1, 1, 0.95))
    # flight information displays hanging over the concourse
    for fx in (-7.5, 0.0, 7.5):
        for side in (-1, 1):
            m.box(fx + side * 0.9, -0.1, 3.75, 0.03, 0.03, 1.5, STEEL, collide=False)
        m.box(fx, -0.1, 2.75, 2.2, 0.12, 0.9, (0.02, 0.02, 0.03), collide=False)
        for k in range(5):
            for s in (1, -1):
                m.box(fx, -0.1 + s * 0.065, 3.05 - k * 0.15, 1.9, 0.005, 0.07, AMBER if k % 2 == 0 else (0.9, 0.9, 0.9),
                      collide=False, emissive=AMBER if k % 2 == 0 else (0.7, 0.7, 0.7))
    # wayfinding signs
    sign(m, -10.5, 3.0, 3.4, 2.0, 0.45, yaw=0.0)
    sign(m, 10.5, -2.6, 3.4, 2.0, 0.45, yaw=0.0)
    sign(m, -6.5, -2.6, 3.4, 2.4, 0.45, yaw=0.0, color=(0.3, 0.75, 1.0))
    return [m, pane]


def apron(rng):
    """Outside the facade: tarmac, taxi lines, two parked aircraft with jet bridges."""
    m = Model("apron")
    y1 = site.BOUNDS[3]
    m.box(0, y1 + 16, -0.06, 60, 32, 0.1, (0.26, 0.27, 0.28), collide=False)
    for gx in site.DEF.GATE_X:
        m.box(gx, y1 + 12, -0.005, 0.15, 22, 0.01, (0.95, 0.8, 0.1), collide=False)     # lead-in line
        m.box(gx, y1 + 4.5, -0.005, 4, 0.15, 0.01, (0.95, 0.8, 0.1), collide=False)     # stop bar
    m.box(0, y1 + 27, -0.005, 60, 0.15, 0.01, (0.95, 0.8, 0.1), collide=False)          # taxiway centreline
    for gx, livery in ((site.DEF.GATE_X[0], (0.1, 0.3, 0.65)), (site.DEF.GATE_X[2], (0.75, 0.12, 0.15))):
        ny = y1 + 6.0                    # nose position
        L, R = 30.0, 1.9
        cy = ny + L / 2
        m.cyl(gx, cy, 3.0, R, L, WHITE, collide=False, roll=math.pi / 2)              # fuselage
        m.sphere(gx, ny, 3.0, R, WHITE)                                                # nose
        m.box(gx, ny + 1.2, 3.55, 1.6, 0.6, 0.35, (0.1, 0.12, 0.15), collide=False)    # cockpit windows
        m.box(gx, cy, 2.55, 0.2, L * 0.95, 0.5, livery, collide=False)                 # cheat line
        m.box(gx, cy + 2, 2.1, 34.0, 4.5, 0.35, (0.9, 0.91, 0.92), collide=False)      # wings
        for ex in (-6.0, 6.0):
            m.cyl(gx + ex, cy + 0.5, 1.2, 0.9, 3.2, (0.8, 0.8, 0.82), collide=False, roll=math.pi / 2)
        m.box(gx, ny + L - 0.5, 6.5, 0.4, 4.0, 5.5, livery, collide=False)             # tail fin
        m.box(gx, ny + L - 0.8, 3.6, 12.0, 2.5, 0.25, (0.9, 0.91, 0.92), collide=False)  # tailplane
        for gy in (ny + 3, cy + 2.5):
            m.cyl(gx, gy, 0.45, 0.45, 0.4, DARK, collide=False, pitch=math.pi / 2)    # gear
        # jet bridge from the gate door to the forward door
        m.box(gx - 2.4, y1 + 3.0, 3.2, 2.4, 6.0, 2.6, (0.72, 0.73, 0.76), collide=False)
        m.box(gx - 2.4, y1 + 5.5, 0.9, 0.5, 0.5, 1.8, STEEL, collide=False)
        # ground support: baggage tug + carts, cones
        m.box(gx + 5, ny + 8, 0.6, 2.0, 1.3, 1.2, (0.95, 0.75, 0.1), collide=False)
        for k in range(3):
            m.box(gx + 5, ny + 10 + k * 2.4, 0.8, 1.6, 2.0, 1.2, (0.4, 0.42, 0.45), collide=False)
        for cx in (gx - 4, gx + 4):
            m.cyl(cx, ny - 0.5, 0.25, 0.15, 0.5, (1.0, 0.4, 0.0), collide=False)
    return m


# ------------------------------------------------------------------ gates
def gate_area(rng):
    m = Model("gate_area")
    d = site.DEF
    for i, gx in enumerate(d.GATE_X):
        m.box(gx, 7.0, 0.55, 1.2, 0.6, 1.1, (0.35, 0.37, 0.42))                       # podium
        m.box(gx, 7.0, 1.12, 1.25, 0.65, 0.04, (0.85, 0.85, 0.86), collide=False)
        m.box(gx + 0.3, 7.05, 1.3, 0.35, 0.03, 0.25, DARK, collide=False)            # monitor
        footprint(gx, 7.0, 1.2, 0.6)
        person(m, gx, 7.55, rng, yaw=-math.pi / 2, bag=False, vest=(0.15, 0.2, 0.4))  # gate agent
        sign(m, gx, 6.4, 3.3, 1.6, 0.5)                                               # GATE n sign
        for k in range(i + 1):                                                        # gate number pips
            m.box(gx - 0.2 + k * 0.2, 6.35, 3.3, 0.12, 0.02, 0.2, (1, 1, 1), collide=False, emissive=(1, 1, 1))
    # seating blocks: rows of chairs on a shared beam, back to back
    for b, (sx0, sx1) in enumerate(d.SEAT_BLOCKS):
        y0, y1 = d.SEAT_Y0, d.SEAT_Y1
        ym = (y0 + y1) / 2
        m.parts.append(f'<collision name="seats{b}"><pose>{(sx0 + sx1) / 2} {ym} 0.45 0 0 0</pose>'
                       f'<geometry><box><size>{sx1 - sx0} {y1 - y0} 0.9</size></box></geometry></collision>')
        footprint((sx0 + sx1) / 2, ym, sx1 - sx0, y1 - y0)
        for row_y, face in ((ym - 0.25, -1), (ym + 0.25, 1)):
            m.box((sx0 + sx1) / 2, row_y, 0.3, sx1 - sx0, 0.06, 0.06, STEEL, collide=False)
            x = sx0 + 0.27
            while x < sx1 - 0.2:
                m.box(x, row_y + face * -0.0, 0.45, 0.46, 0.44, 0.06, (0.18, 0.28, 0.45), collide=False)
                m.box(x, row_y + face * 0.2, 0.72, 0.46, 0.05, 0.5, (0.18, 0.28, 0.45), collide=False)
                if rng.random() < 0.22:
                    person(m, x, row_y - face * 0.05, rng, yaw=-face * math.pi / 2, sitting=True)
                x += 0.5
        for lx in (sx0 + 0.1, sx1 - 0.1):
            m.box(lx, ym, 0.22, 0.06, 0.9, 0.44, STEEL, collide=False)
    return m


# ------------------------------------------------------------------ concourse
def concourse(rng):
    m = Model("concourse")
    d = site.DEF
    for kind, x0, x1, y0, y1, label in d.KIOSKS:
        cx, cy, w, h = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
        footprint(cx, cy, w, h)
        if kind == "cafe":
            m.parts.append(f'<collision name="cafe"><pose>{cx} {cy} 0.5 0 0 0</pose>'
                           f'<geometry><box><size>{w} {h} 1.0</size></box></geometry></collision>')
            m.box(x0 + 0.9, cy, 0.55, 1.6, h - 0.1, 1.1, (0.35, 0.22, 0.14), collide=False)      # counter
            m.box(x0 + 0.9, y1 - 0.1, 2.0, 1.6, 0.06, 0.6, SIGN_BG, collide=False)              # menu board
            m.box(x0 + 0.9, y1 - 0.14, 2.0, 1.4, 0.01, 0.45, (0.95, 0.85, 0.6), collide=False, emissive=(0.8, 0.65, 0.4))
            m.box(x0 + 0.6, cy, 1.25, 0.4, 0.35, 0.35, (0.75, 0.75, 0.78), collide=False)       # espresso machine
            person(m, x0 + 0.9, y1 - 0.35, rng, yaw=-math.pi / 2, bag=False, vest=(0.1, 0.1, 0.1))
            for tx in (x0 + 2.4, x0 + 3.4):
                for ty in (cy - 0.25, cy + 0.25):
                    m.cyl(tx, ty, 0.72, 0.28, 0.04, WHITE, collide=False)
                    m.cyl(tx, ty, 0.36, 0.03, 0.72, STEEL, collide=False)
                    if rng.random() < 0.5:
                        person(m, tx - 0.35, ty, rng, yaw=0.0, sitting=True)
            # low glass barrier around the seating
            for gx in (x0 + 1.8, x1 - 0.02):
                m.box(gx, cy, 0.45, 0.03, h, 0.9, GLASS, collide=False)
            sign(m, cx, y0 - 0.02, 2.6, 1.6, 0.35, color=(0.95, 0.7, 0.4))
        elif kind == "desk":
            m.box(cx, cy, 0.5, w, h, 1.0, (0.85, 0.86, 0.88))
            m.box(cx, cy, 1.02, w + 0.05, h + 0.05, 0.04, (0.3, 0.55, 0.85), collide=False)
            person(m, cx, y1 + 0.25, rng, yaw=-math.pi / 2, bag=False, vest=(0.2, 0.45, 0.8))
            m.cyl(cx, cy, 2.6, 0.35, 0.06, (0.2, 0.5, 0.9), collide=False, roll=math.pi / 2, emissive=(0.1, 0.4, 0.9))
            m.box(cx, cy - 0.04, 2.6, 0.08, 0.02, 0.35, (1, 1, 1), collide=False, emissive=(1, 1, 1))   # "i"
        else:   # shop: back + side walls, glass front, shelves of goods
            m.parts.append(f'<collision name="shop{x0:.0f}"><pose>{cx} {cy} 1.5 0 0 0</pose>'
                           f'<geometry><box><size>{w} {h} 3.0</size></box></geometry></collision>')
            for sx in (x0, x1):
                m.box(sx, cy, 1.5, 0.1, h, 3.0, (0.9, 0.9, 0.9), collide=False)
            m.box(cx, y1, 1.5, w, 0.1, 3.0, (0.9, 0.9, 0.9), collide=False)
            m.box(cx, y0, 2.8, w, 0.1, 0.4, SIGN_BG, collide=False)
            m.box(cx, y0 - 0.06, 2.8, w * 0.6, 0.01, 0.2, (1, 1, 1), collide=False, emissive=(0.9, 0.9, 0.9))
            for k in range(3):                                            # shelves with products
                sy = y1 - 0.3 - k * 0.35
                m.box(cx, sy, 0.6, w - 0.4, 0.25, 1.2, (0.75, 0.72, 0.68), collide=False)
                x = x0 + 0.3
                while x < x1 - 0.3:
                    c = rng.choice([(0.8, 0.2, 0.2), (0.2, 0.4, 0.8), (0.9, 0.75, 0.2), (0.3, 0.6, 0.35),
                                    (0.6, 0.3, 0.6), (0.95, 0.95, 0.95)])
                    hh = rng.uniform(0.15, 0.3)
                    m.box(x, sy, 1.2 + hh / 2, 0.12, 0.18, hh, c, collide=False)
                    x += 0.3
            person(m, cx + 0.8, y0 + 0.35, rng, yaw=math.pi / 2, bag=rng.random() < 0.5)
    # planters with small trees
    for i, (x, y) in enumerate(d.PLANTERS):
        m.cyl(x, y, 0.3, 0.35, 0.6, (0.35, 0.33, 0.32))
        m.cyl(x, y, 1.0, 0.05, 1.0, (0.35, 0.22, 0.12), collide=False)
        for dz, rr in ((1.5, 0.45), (1.85, 0.32)):
            m.sphere(x, y, dz, rr, (0.2, 0.5, 0.25))
        footprint(x, y, 0.7, 0.7)
    for x, y in d.COLUMNS:
        m.cyl(x, y, WALL_H / 2, 0.25, WALL_H, (0.88, 0.88, 0.88))
        footprint(x, y, 0.5, 0.5)
    # rows of luggage trolleys along the side walls
    for tx, ty0, ty1 in d.TROLLEY_ROWS:
        m.parts.append(f'<collision name="trolleys{tx:.0f}"><pose>{tx} {(ty0 + ty1) / 2} 0.5 0 0 0</pose>'
                       f'<geometry><box><size>0.3 {ty1 - ty0} 1.0</size></box></geometry></collision>')
        footprint(tx, (ty0 + ty1) / 2, 0.3, ty1 - ty0)
        y = ty0 + 0.15
        while y < ty1:
            m.box(tx, y, 0.12, 0.3, 0.55, 0.03, STEEL, collide=False)
            m.box(tx, y + 0.25, 0.55, 0.3, 0.03, 0.9, STEEL, collide=False)
            m.box(tx, y + 0.27, 1.0, 0.34, 0.04, 0.04, (0.85, 0.15, 0.15), collide=False)
            y += 0.22
    # pedestrian crossings where people walk over the robot lanes
    for route in site.PASSENGER_ROUTES.values():
        (ax, ay), (bx, by) = route[0], route[-1]
        for lane, (ly, xs) in site.LANES.items():
            if abs(ax - bx) < 0.01 and min(ay, by) < ly < max(ay, by):     # vertical walk crosses lane
                for k in range(-2, 3):
                    m.box(ax + k * 0.3, ly, 0.003, 0.16, 0.9, 0.004, WHITE, collide=False)
        if abs(ay - by) < 0.01:                                            # horizontal walk crosses connectors
            for x, _lanes in d.CROSS:
                if min(ax, bx) < x < max(ax, bx):
                    for k in range(-2, 3):
                        m.box(x, ay + k * 0.3, 0.003, 0.9, 0.16, 0.004, WHITE, collide=False)
    # subtle robot route markings (thin light lines) and docking guides
    for a, b in sorted({tuple(sorted((a, b))) for a in site.EDGES for b in site.EDGES[a]}):
        (x1, y1), (x2, y2) = site.NODES[a], site.NODES[b]
        dock = a in site.CHARGERS or b in site.CHARGERS
        m.box((x1 + x2) / 2, (y1 + y2) / 2, 0.003, math.hypot(x2 - x1, y2 - y1), 0.05, 0.004,
              (0.2, 0.55, 0.95) if dock else (0.62, 0.64, 0.68), collide=False, yaw=math.atan2(y2 - y1, x2 - x1))
    return m


# ------------------------------------------------------------------ check-in + service
def checkin(rng):
    m = Model("checkin_hall")
    d = site.DEF
    y0 = site.BOUNDS[1]
    # main baggage belt along the south wall, with bags
    m.box(5.0, -7.65, 0.35, 10.6, 0.5, 0.06, (0.12, 0.12, 0.13))
    m.box(5.0, -7.65, 0.18, 10.6, 0.45, 0.3, STEEL, collide=False)
    for s in (-1, 1):
        m.box(5.0, -7.65 + s * 0.26, 0.42, 10.6, 0.03, 0.12, STEEL, collide=False)
    footprint(5.0, -7.65, 10.6, 0.5)
    x = 0.3
    while x < 10.0:
        if rng.random() < 0.5:
            hh = rng.uniform(0.2, 0.35)
            m.box(x, -7.65, 0.38 + hh / 2, rng.uniform(0.35, 0.55), 0.3, hh, rng.choice(LUGGAGE), collide=False,
                  yaw=rng.uniform(-0.3, 0.3))
        x += 0.7
    # counters, each with a scale/feeder belt, monitor, agent and an overhead number sign
    for i, cx in enumerate(d.COUNTER_X):
        cy = d.COUNTER_Y
        m.box(cx - 0.3, cy, 0.55, 0.8, 0.7, 1.1, (0.82, 0.82, 0.85))
        m.box(cx - 0.3, cy, 1.12, 0.85, 0.75, 0.04, (0.3, 0.32, 0.36), collide=False)
        m.box(cx + 0.4, cy, 0.25, 0.6, 0.7, 0.5, (0.2, 0.2, 0.22))                       # bag scale
        m.box(cx + 0.4, cy, 0.51, 0.55, 0.65, 0.02, (0.1, 0.1, 0.1), collide=False)
        m.box(cx - 0.3, cy - 0.1, 1.3, 0.35, 0.03, 0.25, DARK, collide=False)
        person(m, cx - 0.3, cy - 0.55, rng, yaw=math.pi / 2, bag=False, vest=(0.15, 0.3, 0.55))
        footprint(cx, cy, 1.4, 0.7)
        sign(m, cx, cy + 0.2, 2.8, 1.0, 0.35)
        for k in range(i + 1):
            m.box(cx - 0.3 + k * 0.15, cy + 0.15, 2.8, 0.1, 0.02, 0.18, (1, 1, 1), collide=False, emissive=(1, 1, 1))
    # oversize baggage drop
    ox0, ox1, oy0, oy1 = d.OVERSIZE
    m.box((ox0 + ox1) / 2, (oy0 + oy1) / 2, 0.45, ox1 - ox0, oy1 - oy0, 0.9, (0.55, 0.57, 0.6))
    m.box((ox0 + ox1) / 2, (oy0 + oy1) / 2, 1.2, 0.6, 0.4, 0.6, (0.12, 0.3, 0.55), collide=False)   # oversize bag
    sign(m, (ox0 + ox1) / 2, oy1 - 0.1, 2.6, 1.0, 0.35, color=(0.3, 0.75, 1.0))
    footprint((ox0 + ox1) / 2, (oy0 + oy1) / 2, ox1 - ox0, oy1 - oy0)
    # queue barriers: chrome posts with retractable belts, serpentine lanes, people waiting
    qx0, qx1, qy0, qy1 = d.QUEUE
    m.parts.append(f'<collision name="queue"><pose>{(qx0 + qx1) / 2} {(qy0 + qy1) / 2} 0.5 0 0 0</pose>'
                   f'<geometry><box><size>{qx1 - qx0} {qy1 - qy0} 1.0</size></box></geometry></collision>')
    footprint((qx0 + qx1) / 2, (qy0 + qy1) / 2, qx1 - qx0, qy1 - qy0)
    rows = [qy0 + (qy1 - qy0) * f for f in (0.0, 0.33, 0.66, 1.0)]
    for ri, ry in enumerate(rows):
        x = qx0
        while x <= qx1 + 0.01:
            m.cyl(x, ry, 0.47, 0.03, 0.94, (0.8, 0.82, 0.85), collide=False)
            m.cyl(x, ry, 0.02, 0.15, 0.04, (0.8, 0.82, 0.85), collide=False)
            x += 1.5
        gap = (qx1 - 0.8, qx1) if ri % 2 == 0 else (qx0, qx0 + 0.8)          # serpentine openings
        a, b = (qx0, gap[0]) if ri % 2 == 0 else (gap[1], qx1)
        m.box((a + b) / 2, ry, 0.9, b - a, 0.02, 0.05, (0.15, 0.3, 0.75), collide=False)
    for _ in range(9):
        lane = rng.randrange(3)
        px = rng.uniform(qx0 + 0.4, qx1 - 0.4)
        py = (rows[lane] + rows[lane + 1]) / 2
        person(m, px, py, rng, yaw=math.pi if lane % 2 == 0 else 0.0)
    return m


def charging_room():
    m = Model("robot_charging")
    d = site.DEF
    x0 = site.BOUNDS[0]
    px1 = -1.8
    # glass partition with frame, enclosing the robot area from the staff room behind
    m.box((x0 + 0.1 + px1) / 2, d.PARTITION_Y, 1.25, px1 - x0 - 0.1, 0.05, 2.5, GLASS, collide=True)
    m.box((x0 + 0.1 + px1) / 2, d.PARTITION_Y, 2.5, px1 - x0 - 0.1, 0.1, 0.06, STEEL, collide=False)
    footprint((x0 + 0.1 + px1) / 2, d.PARTITION_Y, px1 - x0 - 0.1, 0.1)
    for x in d.CHARGER_X:
        m.box(x, d.CHARGER_Y, 0.35, 0.6, 0.3, 0.7, (0.2, 0.2, 0.22))
        m.box(x, d.CHARGER_Y + 0.16, 0.55, 0.3, 0.02, 0.12, (0.1, 0.9, 0.3), collide=False, emissive=(0.0, 0.7, 0.2))
        m.box(x, d.CHARGER_Y + 0.25, 0.12, 0.4, 0.2, 0.06, (0.6, 0.5, 0.1), collide=False)
        footprint(x, d.CHARGER_Y, 0.6, 0.3)
        for dx in (-0.5, 0.5):
            m.box(x + dx, (d.DOCK_Y + d.CHARGER_Y) / 2, 0.003, 0.06, 1.3, 0.004, (0.2, 0.55, 0.95), collide=False)
    sign(m, -6.5, d.PARTITION_Y + 0.08, 2.2, 2.4, 0.4, color=(0.3, 0.75, 1.0))
    # staff room behind the glass: lockers
    for k in range(8):
        m.box(-11.2 + k * 0.6, -7.6, 1.0, 0.55, 0.45, 2.0, (0.45, 0.5, 0.55), collide=False)
    return m


def build_airport(seed=7):
    FOOTPRINTS.clear()
    rng = random.Random(seed)
    models = building() + [apron(rng), gate_area(rng), concourse(rng), checkin(rng), charging_room()]
    lights = point_lights([(x, y) for x in (-8, 0, 8) for y in (-5, 0.5, 5.5)], z=4.3, rng=14)
    return world_sdf(site.NAME, models, lights, ambient=0.62, background=(0.62, 0.78, 0.95))
