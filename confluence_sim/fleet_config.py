"""Physical robot models for the Gazebo simulation (body, wheels, lidar).

Each robot is deliberately different (size, colour, speed) - a heterogeneous
fleet, which is what Confluence's integration layer is meant to handle.
"""

# Physical robots spawned in Gazebo. Which vendor/interface each one has is
# decided by the simulated vendor software (vendors/run.py); Confluence learns
# about them only from confluence/config/fleet_<site>.yaml, never from here.
# Spawn points and body styling come from the active site (confluence/site.py).
from confluence import site

ROBOTS = [
    {"name": "r1", "kind": "amr_small", "spawn": (-6.0, 0.0),
     "size": (0.50, 0.40, 0.12), "color": (0.1, 0.4, 0.9, 1.0), "max_v": 0.5},
    {"name": "r2", "kind": "amr_large", "spawn": (6.0, 4.8),
     "size": (0.80, 0.60, 0.18), "color": (0.95, 0.5, 0.1, 1.0), "max_v": 0.35},
    {"name": "r3", "kind": "tugger", "spawn": (0.0, -3.3),
     "size": (0.60, 0.45, 0.25), "color": (0.2, 0.75, 0.3, 1.0), "max_v": 0.4},
    {"name": "r4", "kind": "forklift_agv", "spawn": (8.1, -5.15),
     "size": (0.90, 0.60, 0.30), "color": (0.85, 0.75, 0.1, 1.0), "max_v": 0.3},
]

PERSON_COLORS = [(0.75, 0.15, 0.2, 1), (0.15, 0.35, 0.7, 1), (0.25, 0.55, 0.3, 1), (0.5, 0.3, 0.6, 1)]


def robots():
    """Robot list for the active site (spawn points + per-site body styling)."""
    kinds = getattr(site.DEF, "ROBOT_KINDS", {})
    out = []
    for r in ROBOTS:
        r = dict(r)
        r["spawn"] = site.SPAWNS.get(r["name"], r["spawn"])
        r["kind"] = kinds.get(r["name"], r["kind"])
        out.append(r)
    return out


def passengers():
    """Walking passengers for the active site: name -> spawn (first waypoint)."""
    return [{"name": n, "spawn": route[0], "route": route, "color": PERSON_COLORS[i % len(PERSON_COLORS)]}
            for i, (n, route) in enumerate(site.PASSENGER_ROUTES.items())]


def guests():
    """Passengers who follow a guide robot (confluence/sites/<site>.py GUESTS)."""
    return [{"name": n, "spawn": cfg["wait"], "cfg": cfg, "robot": cfg["robot"], "color": (0.95, 0.75, 0.1, 1)}
            for n, cfg in site.GUESTS.items()]


WHEEL_RADIUS = 0.08
WHEEL_WIDTH = 0.05


def _box_inertia(m, x, y, z):
    return (m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z), m / 12.0 * (x * x + y * y))


def robot_sdf(robot):
    """Return an SDF string for a simple differential-drive robot."""
    name = robot["name"]
    L, W, H = robot["size"]
    r, g, b, a = robot["color"]
    sep = W + WHEEL_WIDTH + 0.02  # wheel separation
    chassis_z = WHEEL_RADIUS + 0.02
    caster_r = 0.04
    caster_z = caster_r - chassis_z  # relative to chassis, touches the ground
    ixx, iyy, izz = _box_inertia(5.0, L, W, H)
    wi = 0.5 * (3 * WHEEL_RADIUS ** 2 + WHEEL_WIDTH ** 2) / 12.0

    def wheel(side, y):
        return f"""
    <link name="{side}_wheel">
      <pose>0 {y} {WHEEL_RADIUS} -1.5707 0 0</pose>
      <inertial><mass>0.5</mass>
        <inertia><ixx>{wi}</ixx><iyy>{wi}</iyy><izz>{0.25 * WHEEL_RADIUS ** 2}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder><radius>{WHEEL_RADIUS}</radius><length>{WHEEL_WIDTH}</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>{WHEEL_RADIUS}</radius><length>{WHEEL_WIDTH}</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
    </link>
    <joint name="{side}_wheel_joint" type="revolute">
      <parent>chassis</parent><child>{side}_wheel</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>"""

    def caster(tag, x):
        return f"""
      <collision name="caster_{tag}">
        <pose>{x} 0 {caster_z} 0 0 0</pose>
        <geometry><sphere><radius>{caster_r}</radius></sphere></geometry>
        <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>
      </collision>"""

    lz = min(0.08, H * 0.6)  # front-bumper lidar height (chassis frame)
    extras = ""
    if robot["kind"] == "forklift_agv":
        # mast + two forks out front, so it reads as a forklift AGV
        extras = f"""
      <visual name="mast"><pose>{L / 2 - 0.05} 0 {H + 0.35} 0 0 0</pose>
        <geometry><box><size>0.06 {W * 0.8} 0.7</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>"""
        for fy in (-W * 0.25, W * 0.25):
            extras += f"""
      <visual name="fork_{'l' if fy > 0 else 'r'}"><pose>{L / 2 + 0.15} {fy} 0.03 0 0 0</pose>
        <geometry><box><size>0.35 0.08 0.03</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>"""
    elif robot["kind"] == "tugger":
        extras = f"""
      <visual name="tow_hitch"><pose>{-L / 2 - 0.06} 0 0.1 0 0 0</pose>
        <geometry><box><size>0.12 0.08 0.05</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>"""
    elif robot["kind"] == "guide":
        # airport guide robot: slim mast with a tablet screen facing forward
        extras = f"""
      <visual name="mast"><pose>-0.05 0 {H + 0.45} 0 0 0</pose>
        <geometry><cylinder><radius>0.05</radius><length>0.9</length></cylinder></geometry>
        <material><ambient>0.85 0.85 0.88 1</ambient><diffuse>0.85 0.85 0.88 1</diffuse></material></visual>
      <visual name="screen"><pose>0.0 0 {H + 0.85} 0 -0.35 0</pose>
        <geometry><box><size>0.03 0.32 0.22</size></box></geometry>
        <material><ambient>0.05 0.25 0.45 1</ambient><diffuse>0.1 0.4 0.8 1</diffuse>
          <emissive>0.05 0.3 0.6 1</emissive></material></visual>"""
    elif robot["kind"] == "scrubber":
        # floor scrubber: clean-water tank on top, brush deck and squeegee underneath
        extras = f"""
      <visual name="tank"><pose>-0.08 0 {H + 0.14} 0 0 0</pose>
        <geometry><box><size>{L * 0.55} {W * 0.85} 0.28</size></box></geometry>
        <material><ambient>0.85 0.85 0.9 1</ambient><diffuse>0.9 0.9 0.95 1</diffuse></material></visual>
      <visual name="brush"><pose>{L * 0.2} 0 -0.07 0 0 0</pose>
        <geometry><cylinder><radius>{W * 0.42}</radius><length>0.03</length></cylinder></geometry>
        <material><ambient>0.1 0.3 0.8 1</ambient><diffuse>0.1 0.3 0.8 1</diffuse></material></visual>
      <visual name="squeegee"><pose>{-L / 2 - 0.03} 0 -0.08 0 0 0</pose>
        <geometry><box><size>0.04 {W + 0.1} 0.02</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>"""
    elif robot["kind"] == "baggage_agv":
        # baggage AGV: flat deck carrying two suitcases
        extras = f"""
      <visual name="deck"><pose>0 0 {H + 0.02} 0 0 0</pose>
        <geometry><box><size>{L * 0.95} {W * 0.95} 0.04</size></box></geometry>
        <material><ambient>0.3 0.3 0.32 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse></material></visual>
      <visual name="bag1"><pose>-0.18 0.05 {H + 0.24} 0 0 0.1</pose>
        <geometry><box><size>0.45 0.3 0.4</size></box></geometry>
        <material><ambient>0.6 0.1 0.15 1</ambient><diffuse>0.7 0.12 0.18 1</diffuse></material></visual>
      <visual name="bag2"><pose>0.22 -0.05 {H + 0.19} 0 0 -0.15</pose>
        <geometry><box><size>0.38 0.28 0.3</size></box></geometry>
        <material><ambient>0.1 0.15 0.2 1</ambient><diffuse>0.12 0.18 0.25 1</diffuse></material></visual>"""
    elif robot["kind"] == "amr_large":
        extras = f"""
      <visual name="tote"><pose>0 0 {H + 0.12} 0 0 0</pose>
        <geometry><box><size>{L * 0.6} {W * 0.7} 0.16</size></box></geometry>
        <material><ambient>0.2 0.3 0.7 1</ambient><diffuse>0.2 0.3 0.7 1</diffuse></material></visual>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="chassis">
      <pose>0 0 {chassis_z} 0 0 0</pose>
      <inertial><mass>5.0</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="body"><pose>0 0 {H / 2} 0 0 0</pose>
        <geometry><box><size>{L} {W} {H}</size></box></geometry></collision>
      <visual name="body"><pose>0 0 {H / 2} 0 0 0</pose>
        <geometry><box><size>{L} {W} {H}</size></box></geometry>
        <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material></visual>
      <visual name="front_marker"><pose>{L / 2 - 0.05} 0 {H + 0.01} 0 0 0</pose>
        <geometry><box><size>0.06 0.06 0.02</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      <visual name="lidar_puck"><pose>{L / 2 + 0.03} 0 {lz} 0 0 0</pose>
        <geometry><cylinder><radius>0.05</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      {extras}
      <sensor name="lidar" type="gpu_lidar">
        <pose>{L / 2 + 0.03} 0 {lz} 0 0 0</pose>
        <topic>/{name}/scan</topic>
        <gz_frame_id>{name}/lidar</gz_frame_id>
        <update_rate>10</update_rate>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <lidar>
          <scan><horizontal><samples>190</samples><resolution>1</resolution>
            <min_angle>-1.65</min_angle><max_angle>1.65</max_angle></horizontal></scan>
          <range><min>0.12</min><max>8.0</max><resolution>0.01</resolution></range>
        </lidar>
      </sensor>
      {caster("front", L / 2 - 0.08)}
      {caster("back", -L / 2 + 0.08)}
    </link>
    {wheel("left", sep / 2)}
    {wheel("right", -sep / 2)}
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>{sep}</wheel_separation>
      <wheel_radius>{WHEEL_RADIUS}</wheel_radius>
      <max_linear_velocity>{robot["max_v"]}</max_linear_velocity>
      <topic>/{name}/cmd_vel</topic>
      <odom_topic>/{name}/odom</odom_topic>
      <frame_id>{name}/odom</frame_id>
      <child_frame_id>{name}/base_link</child_frame_id>
      <odom_publish_frequency>20</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
"""


def person_sdf(p):
    """A walking passenger: small hidden diff-drive base under a person-shaped
    visual (legs, torso, head, a roller bag). Robots' lidars see the legs."""
    name = p["name"]
    r, g, b, a = p["color"]
    wr, sep = 0.05, 0.3
    wi = 0.5 * (3 * wr ** 2 + 0.03 ** 2) / 12.0

    def wheel(side, y):
        return f"""
    <link name="{side}_wheel"><pose>0 {y} {wr} -1.5707 0 0</pose>
      <inertial><mass>0.3</mass><inertia><ixx>{wi}</ixx><iyy>{wi}</iyy><izz>{0.5 * 0.3 * wr ** 2}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder><radius>{wr}</radius><length>0.03</length></cylinder></geometry></collision>
    </link>
    <joint name="{side}_wheel_joint" type="revolute"><parent>body</parent><child>{side}_wheel</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>"""

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="body">
      <pose>0 0 0.07 0 0 0</pose>
      <inertial><pose>0 0 0.05 0 0 0</pose><mass>6.0</mass>
        <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="base"><pose>0 0 0.12 0 0 0</pose>
        <geometry><cylinder><radius>0.2</radius><length>0.2</length></cylinder></geometry></collision>
      <collision name="caster_f"><pose>0.12 0 -0.03 0 0 0</pose><geometry><sphere><radius>0.04</radius></sphere></geometry>
        <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
      <collision name="caster_b"><pose>-0.12 0 -0.03 0 0 0</pose><geometry><sphere><radius>0.04</radius></sphere></geometry>
        <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
      <visual name="leg_l"><pose>0 0.08 0.36 0 0 0</pose><geometry><cylinder><radius>0.06</radius><length>0.86</length></cylinder></geometry>
        <material><ambient>0.15 0.15 0.2 1</ambient><diffuse>0.15 0.15 0.2 1</diffuse></material></visual>
      <visual name="leg_r"><pose>0 -0.08 0.36 0 0 0</pose><geometry><cylinder><radius>0.06</radius><length>0.86</length></cylinder></geometry>
        <material><ambient>0.15 0.15 0.2 1</ambient><diffuse>0.15 0.15 0.2 1</diffuse></material></visual>
      <visual name="torso"><pose>0 0 1.05 0 0 0</pose><geometry><box><size>0.22 0.4 0.6</size></box></geometry>
        <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material></visual>
      <visual name="head"><pose>0 0 1.5 0 0 0</pose><geometry><sphere><radius>0.11</radius></sphere></geometry>
        <material><ambient>0.8 0.62 0.48 1</ambient><diffuse>0.8 0.62 0.48 1</diffuse></material></visual>
      <visual name="bag"><pose>-0.3 -0.2 0.25 0 0 0</pose><geometry><box><size>0.2 0.35 0.5</size></box></geometry>
        <material><ambient>0.2 0.2 0.22 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse></material></visual>
      <visual name="handle"><pose>-0.3 -0.2 0.68 0 0 0</pose><geometry><box><size>0.02 0.25 0.36</size></box></geometry>
        <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
    </link>
    {wheel("left", sep / 2)}
    {wheel("right", -sep / 2)}
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
      <wheel_separation>{sep}</wheel_separation><wheel_radius>{wr}</wheel_radius>
      <max_linear_velocity>1.0</max_linear_velocity>
      <topic>/{name}/cmd_vel</topic><odom_topic>/{name}/odom</odom_topic>
      <frame_id>{name}/odom</frame_id><child_frame_id>{name}/base_link</child_frame_id>
      <odom_publish_frequency>20</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
"""
