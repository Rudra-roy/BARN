#!/usr/bin/env python3
"""Generate DynaBARN dynamic-obstacle worlds for the BARN ROS2 evaluator.

The evaluator (jackal_helper/scripts/barn_runner.py) spawns the Jackal for
world_idx 300-359 at INIT_POSITION [11, 0, 3.14] (facing -x) and drives it to
GOAL_POSITION offset [-20, 0] => goal (-9, 0). So the robot travels from x=+11
to x=-9 along y=0, and EVERY obstacle must sit ahead of it (x < 11) near that
path to matter.

Style follows the DynaBARN paper: a wide open arena (blue boundary walls) with
many small red cylinders scattered across it, each sweeping a short path so it
drifts on and off the robot's route. Small radius (0.20 m) so they read as
tidy obstacles, not giant pillars. Obstacles start inside the arena (visible
from t=0). SDF structure is copied from the world that loads in gz-sim 8; only
geometry/placement change.

Regenerate: python3 tools/generate_dynabarn_worlds.py
Then install: bash tools/setup_dynabarn.sh
"""
import os

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT_DIR = os.path.join(REPO_ROOT, "worlds", "DynaBARN")

# Arena geometry (metres). Robot runs y=0 from x=+11 to x=-9.
HALF_WIDTH = 6.0            # blue boundary walls at y = +/- HALF_WIDTH (12 m wide)
WALL_CENTER_X = 1.0
WALL_LENGTH = 26.0          # spans x in [-12, 14], covering start and goal
# Obstacles live ahead of the robot, between the goal and just shy of the start.
X_NEAR = 9.0               # closest obstacle to the start (x=11)
X_FAR = -7.0              # closest obstacle to the goal (x=-9)
OBST_RADIUS = 0.20
OBST_LENGTH = 0.80
SWEEP_AMP = 1.2            # each cylinder oscillates +/- this about its centre

# Scatter of lane offsets cycled across obstacles: several near y=0 (challenge
# the robot) plus some further out (visual density, mostly beyond the MPC gate).
Y_CENTERS = [0.0, -2.5, 1.5, -0.8, 3.2, 0.8, -3.5, 2.4, -1.6, 3.8, -0.3, -4.0]

HEADER = """<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.6">
  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" /><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" /><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" /><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact" /><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" /><plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat" /><light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose frame="">0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>
    <model name="ground_plane">
      <static>1</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode />
              </torsional>
            </friction>
            <contact>
              <ode />
            </contact>
            <bounce />
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name="visual">
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic" />
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

FOOTER = "  </world>\n</sdf>\n"


def wall(name, cx, cy, length):
    return f"""    <model name="{name}">
      <static>1</static>
      <pose frame="">{cx:.6f} {cy:.6f} 0.500000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{length:.6f} 0.200000 1.000000</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact><collide_bitmask>0x01</collide_bitmask><ode /></contact>
            <bounce />
            <friction><torsional><ode /></torsional><ode /></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{length:.6f} 0.200000 1.000000</size>
            </box>
          </geometry>
          <material>
            <ambient>0.10 0.10 0.85 1</ambient>
            <diffuse>0.15 0.15 0.95 1</diffuse>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
        <sensor name="sensor_contact" type="contact"><contact><collision>collision</collision></contact></sensor>
      </link>
      <plugin filename="gz-sim-touchplugin-system" name="gz::sim::systems::TouchPlugin"><target>robot</target><namespace>robot</namespace><time>0.001</time><enabled>true</enabled></plugin>
    </model>
"""


def cylinder(name, x, y0, y1, y2, speed, bitmask):
    # Small solid cylinder; gravity off + velocity control, so exact inertia is
    # not critical. Red to match the DynaBARN reference.
    return f"""    <model name="{name}">
      <pose frame="">{x:.6f} {y0:.6f} 0.400000 0 0 0</pose>
      <link name="link">
        <gravity>0</gravity>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.6333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.6333</iyy>
            <iyz>0</iyz>
            <izz>0.2000</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{OBST_RADIUS:.3f}</radius>
              <length>{OBST_LENGTH:.3f}</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact><collide_bitmask>{bitmask}</collide_bitmask><ode /></contact>
            <bounce />
            <friction><torsional><ode /></torsional><ode /></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{OBST_RADIUS:.3f}</radius>
              <length>{OBST_LENGTH:.3f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.850 0.130 0.130 1</ambient>
            <diffuse>0.900 0.170 0.170 1</diffuse>
          </material>
        </visual>
        <sensor name="sensor_contact" type="contact"><contact><collision>collision</collision></contact></sensor>
      </link>
      <plugin filename="barn_dynamic_obstacle" name="barn::sim::DynamicObstacle">
        <speed>{speed:.2f}</speed>
        <loop>true</loop>
        <waypoint>{x:.3f} {y0:.3f}</waypoint>
        <waypoint>{x:.3f} {y1:.3f}</waypoint>
        <waypoint>{x:.3f} {y2:.3f}</waypoint>
      </plugin>
      <plugin filename="gz-sim-touchplugin-system" name="gz::sim::systems::TouchPlugin"><target>robot</target><namespace>robot</namespace><time>0.001</time><enabled>true</enabled></plugin>
    </model>
"""


# (filename, count, (speed_lo, speed_hi))
TIERS = {
    "world_0.world": (4, (0.4, 0.7)),
    "world_1.world": (8, (0.6, 1.0)),
    "world_2.world": (12, (0.9, 1.5)),
}


def build_world(count, speed_lo, speed_hi):
    parts = [HEADER,
             wall("wall_south", WALL_CENTER_X, -HALF_WIDTH, WALL_LENGTH),
             wall("wall_north", WALL_CENTER_X, HALF_WIDTH, WALL_LENGTH)]
    span = X_NEAR - X_FAR
    for i in range(count):
        # Spread obstacles from near the start (X_NEAR) toward the goal (X_FAR).
        frac = i / max(1, count - 1)
        x = X_NEAR - span * frac
        yc = Y_CENTERS[i % len(Y_CENTERS)]
        speed = speed_lo + (speed_hi - speed_lo) * ((i * 7) % max(1, count)) / max(1, count)
        # Alternate sweep direction and clamp the sweep inside the arena.
        amp = SWEEP_AMP
        lo = max(-HALF_WIDTH + 0.5, yc - amp)
        hi = min(HALF_WIDTH - 0.5, yc + amp)
        if i % 2 == 0:
            y0, y1, y2 = lo, yc, hi
        else:
            y0, y1, y2 = hi, yc, lo
        bitmask = f"0x{(1 << ((i % 15) + 1)):04x}"
        parts.append(cylinder(f"obstacle_{i}", round(x, 3), y0, y1, y2,
                              speed, bitmask))
    parts.append(FOOTER)
    return "".join(parts)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for fname, (count, (lo, hi)) in TIERS.items():
        path = os.path.join(OUT_DIR, fname)
        with open(path, "w", encoding="utf-8") as f:
            f.write(build_world(count, lo, hi))
        print(f"wrote {path} ({count} moving cylinders, r={OBST_RADIUS} m)")


if __name__ == "__main__":
    main()
