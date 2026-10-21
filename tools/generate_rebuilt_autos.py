"""Generates the 20 REBUILT autos ("R26 ...") as PathPlanner .path and .auto files.

Run from the project folder:   python tools/generate_rebuilt_autos.py

Everything lives in one PathPlanner folder, "Rebuilt 20". Re-running replaces every file in that
folder, so make changes here (a pose, a speed, a routine) rather than in the PathPlanner GUI, or
they will be overwritten. Other autos and paths are never touched. After regenerating, run
./gradlew test: AutoPathsTest checks the routes against the game manual and AutoCatalogTest checks
each routine still fits the 20 s AUTO period.

All coordinates are BLUE-side field metres, WPILib convention (blue wall at x=0). PathPlanner
mirrors them for red at runtime. Geometry comes from the 2026 manual and the official AprilTag
layout:

  ROBOT STARTING LINE far edge 4.028 m from the wall; BUMPS start at 4.062 m (G303: bumpers must
  overlap the line and not touch a BUMP). HUB centre (4.626, 4.035). TRENCH lanes centred on
  y=0.834 and y=7.235. OUTPOST/CORRAL opening centred on tag 29, y=0.666. TOWER base x 0..1.148,
  y 3.25..4.24. DEPOT x 0..0.686, y ~5.40..6.47. Neutral FUEL starts at x 7.36.
"""

import json
import math
import os
import sys

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..")
PATHS = os.path.join(ROOT, "src/main/deploy/pathplanner/paths")
AUTOS = os.path.join(ROOT, "src/main/deploy/pathplanner/autos")
FOLDER = "Rebuilt 20"

HUB = (4.626, 4.035)
LANE_L = 0.834
LANE_H = 8.069 - 0.834

TRANSIT = 3.4    # m/s on open carpet (62% of the 5.45 m/s top speed in settings.json)
ACCEL = 3.2      # m/s^2
SWEEP = 1.5      # m/s while intaking a row of FUEL
WALL = 1.0       # m/s for the last stretch into the wall


def back_to_hub(x, y):
    """Heading that points the robot's BACK (turret and camera) at the HUB."""
    return round(math.degrees(math.atan2(HUB[1] - y, HUB[0] - x)) - 180.0, 1)


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


# ---- Poses --------------------------------------------------------------------------------------

def shot(x, y):
    return (x, y, wrap(back_to_hub(x, y)))


START = {
    "L": (3.57, LANE_L, 180.0),
    "BL": (3.57, 2.55, 180.0),
    "C": (3.55, 4.035, 180.0),
    "BH": (3.57, 5.52, 180.0),
    "H": (3.57, LANE_H, 180.0),
}

# Same spots, squared the other way: intake toward the field, for autos that drive straight out.
START_OUT = {k: (v[0], v[1], 0.0) for k, v in START.items()}

SHOT = {
    "L": shot(2.90, LANE_L),
    "H": shot(2.90, LANE_H),
    "BL": shot(2.75, 2.45),
    "BH": shot(2.75, 5.62),
    "C": (2.30, 4.035, 180.0),
    "O": shot(1.55, 1.40),
    "D": shot(1.90, 6.37),
}

# ---- Waypoint helpers ---------------------------------------------------------------------------
# A waypoint is (x, y, heading or None, tangent-degrees or None).


def wp(x, y, h=None, t=None):
    return (x, y, h, t)


def lane_out(side):
    """Through the trench from the alliance side, heading held square inside the lane."""
    y = LANE_L if side == "L" else LANE_H
    return [wp(3.55, y, None, 0), wp(5.95, y, None, 0)]


def lane_in(side):
    y = LANE_L if side == "L" else LANE_H
    return [wp(5.95, y, None, 180), wp(3.55, y, None, 180)]


NZ = {
    # sweep rows along x=7.05, front (intake) facing the fuel
    "L": [wp(7.05, 1.70, 0.0, 90), wp(7.05, 2.70, 0.0, 90)],
    "LC": [wp(7.05, 1.70, 0.0, 90), wp(7.05, 3.20, 0.0, 90)],
    "H": [wp(7.05, 6.37, 0.0, -90), wp(7.05, 5.37, 0.0, -90)],
    "HC": [wp(7.05, 6.37, 0.0, -90), wp(7.05, 4.87, 0.0, -90)],
    "LX": [wp(7.05, 1.70, 0.0, 90), wp(7.05, 3.70, 0.0, 90)],
    "HX": [wp(7.05, 6.37, 0.0, -90), wp(7.05, 4.37, 0.0, -90)],
}

CHUTE = [wp(1.75, 0.95, 180.0, 180), wp(0.66, 0.72, 180.0, 180)]
DEPOT = [wp(1.20, 5.15, 180.0, 90), wp(1.20, 6.65, 180.0, 90)]
CORRAL = wp(0.50, 0.666, 180.0, 180)


def pose_wp(pose, t=None):
    return wp(pose[0], pose[1], pose[2], t)


# ---- Path writer --------------------------------------------------------------------------------

def clear_folder(directory, extension):
    """Removes this generator's previous output, so renamed or dropped legs do not linger."""
    for name in os.listdir(directory):
        if name.endswith(extension):
            full = os.path.join(directory, name)
            try:
                with open(full, encoding="utf-8") as f:
                    if json.load(f).get("folder") == FOLDER:
                        os.remove(full)
            except (OSError, ValueError):
                pass



# This generator owns the folder: clear its old output first, so a renamed or dropped leg does not
# linger as a stale file.
clear_folder(PATHS, ".path")
clear_folder(AUTOS, ".auto")

written = {}


def tangent(points, i):
    x, y, _, t = points[i]
    if t is not None:
        return math.radians(t)
    if i == 0:
        nx, ny = points[1][0], points[1][1]
        return math.atan2(ny - y, nx - x)
    if i == len(points) - 1:
        px, py = points[i - 1][0], points[i - 1][1]
        return math.atan2(y - py, x - px)
    px, py = points[i - 1][0], points[i - 1][1]
    nx, ny = points[i + 1][0], points[i + 1][1]
    a = math.atan2(y - py, x - px)
    b = math.atan2(ny - y, nx - x)
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def write_path(name, points, slow=None, wall_end=False):
    """slow: list of (from_index, to_index, speed) segments driven gently (intake rows, walls)."""
    if name in written:
        return name
    waypoints = []
    for i, (x, y, _, _) in enumerate(points):
        theta = tangent(points, i)
        entry = {"anchor": {"x": round(x, 4), "y": round(y, 4)}, "prevControl": None,
                 "nextControl": None, "isLocked": False, "linkedName": None}
        if i > 0:
            d = 0.38 * math.dist(points[i - 1][:2], (x, y))
            entry["prevControl"] = {"x": round(x - d * math.cos(theta), 4),
                                    "y": round(y - d * math.sin(theta), 4)}
        if i < len(points) - 1:
            d = 0.38 * math.dist(points[i + 1][:2], (x, y))
            entry["nextControl"] = {"x": round(x + d * math.cos(theta), 4),
                                    "y": round(y + d * math.sin(theta), 4)}
        waypoints.append(entry)

    rotation_targets = []
    for i in range(1, len(points) - 1):
        if points[i][2] is not None:
            rotation_targets.append({"waypointRelativePos": i, "rotationDegrees": points[i][2]})

    zones = []
    for (a, b, speed) in (slow or []):
        zones.append({"name": "Slow", "minWaypointRelativePos": a, "maxWaypointRelativePos": b,
                      "constraints": {"maxVelocity": speed, "maxAcceleration": ACCEL,
                                      "maxAngularVelocity": 540.0, "maxAngularAcceleration": 1000.0,
                                      "nominalVoltage": 12.0, "unlimited": False}})

    start_h = points[0][2] if points[0][2] is not None else 0.0
    end_h = points[-1][2] if points[-1][2] is not None else start_h
    path = {
        "version": "2025.0",
        "waypoints": waypoints,
        "rotationTargets": rotation_targets,
        "constraintZones": zones,
        "pointTowardsZones": [],
        "eventMarkers": [],
        "globalConstraints": {"maxVelocity": TRANSIT, "maxAcceleration": ACCEL,
                              "maxAngularVelocity": 540.0, "maxAngularAcceleration": 1000.0,
                              "nominalVoltage": 12.0, "unlimited": False},
        "goalEndState": {"velocity": 0, "rotation": end_h},
        "reversed": False,
        "folder": FOLDER,
        "idealStartingState": {"velocity": 0, "rotation": start_h},
        "useDefaultConstraints": False,
    }
    with open(os.path.join(PATHS, name + ".path"), "w", encoding="utf-8") as f:
        json.dump(path, f, indent=2)
    written[name] = path
    return name


# ---- Leg library --------------------------------------------------------------------------------
# Each leg returns (path name, intake while driving?).

def leg(name, points, intake=False, slow=None):
    return ("path", write_path(name, points, slow), intake)


def start_to_shot(s):
    st, sh = START[s], SHOT[s]
    return leg(f"{s} Start to Shot", [pose_wp(st), pose_wp(sh)])


def to_neutral(origin_name, origin, side, row):
    """From a pose on the alliance side, under the trench, then sweep a row of neutral FUEL."""
    lane = lane_out(side)
    on_lane = abs(origin[1] - lane[0][1]) < 0.05 and origin[0] > lane[0][0]
    # Starting on the lane already: no need to back up to the lane entry first.
    first = wp(origin[0], origin[1], origin[2], 0) if on_lane else pose_wp(origin)
    pts = [first] + (lane[1:] if on_lane else lane) + NZ[row]
    # Square to the lane before the trench, and turn to face the FUEL once out of it.
    sq = 0.0
    for i in range(1, len(pts) - 2):
        pts[i] = (pts[i][0], pts[i][1], sq, pts[i][3])
    n = len(pts)
    return leg(f"{origin_name} to Neutral {row}", pts, True, [(n - 2, n - 1, SWEEP)])


def from_neutral(side, row, dest_name, dest):
    end = NZ[row][-1]
    pts = [wp(end[0], end[1], 0.0, None)] + lane_in(side) + [pose_wp(dest)]
    pts[1] = (pts[1][0], pts[1][1], 0.0, pts[1][3])
    pts[2] = (pts[2][0], pts[2][1], 0.0, pts[2][3])
    return leg(f"Neutral {row} to {dest_name}", pts)


def to_chute(origin_name, origin, via=None):
    pts = [pose_wp(origin)] + (via or []) + CHUTE
    return leg(f"{origin_name} to Chute", pts, True, [(len(pts) - 2, len(pts) - 1, WALL)])


def chute_to(dest_name, dest):
    end = CHUTE[-1]
    return leg(f"Chute to {dest_name}", [wp(end[0], end[1], 180.0, 0), pose_wp(dest)])


# Coming from below, the robot swings wide of the TOWER and sweeps the DEPOT upward; coming from
# above it sweeps downward. Either way the row covers the DEPOT's full 42 in.
DEPOT_UP = [wp(1.85, 4.95, 180.0, 135), wp(1.20, 5.30, 180.0, 90), wp(1.20, 6.75, 180.0, 90)]
DEPOT_DOWN = [wp(1.55, 6.95, 180.0, 180), wp(1.20, 6.55, 180.0, -90), wp(1.20, 5.30, 180.0, -90)]


def depot_row(origin):
    return DEPOT_DOWN if origin[1] > 6.0 else DEPOT_UP


def to_depot(origin_name, origin):
    pts = [pose_wp(origin)] + depot_row(origin)
    return leg(f"{origin_name} to Depot", pts, True, [(len(pts) - 2, len(pts) - 1, SWEEP)])


def depot_to(dest_name, dest, via=None, came_from=None):
    """From the end of the DEPOT row the robot finished on, which depends on where it came from."""
    end = depot_row(came_from)[-1]
    direction = "Up" if end[1] > 6.0 else "Down"
    pts = [wp(end[0], end[1], 180.0, None)] + (via or []) + [pose_wp(dest)]
    return leg(f"Depot {direction} to {dest_name}", pts)


def to_corral(origin_name, origin, via=None):
    pts = [pose_wp(origin)] + (via or []) + [wp(1.30, 0.70, 180.0, 180), CORRAL]
    return leg(f"{origin_name} to Corral", pts, False, [(len(pts) - 2, len(pts) - 1, WALL)])


def corral_to(dest_name, dest, via=None):
    pts = [wp(CORRAL[0], CORRAL[1], 180.0, 0)] + (via or []) + [pose_wp(dest)]
    return leg(f"Corral to {dest_name}", pts)


def corral_to_neutral(side, row):
    origin = (CORRAL[0], CORRAL[1], 180.0)
    pts = [wp(origin[0], origin[1], 180.0, 0), wp(2.20, LANE_L, 180.0, 0)] + lane_out(side) + NZ[row]
    pts[2] = (pts[2][0], pts[2][1], 180.0, pts[2][3])
    pts[3] = (pts[3][0], pts[3][1], 180.0, pts[3][3])
    n = len(pts)
    return leg(f"Corral to Neutral {row}", pts, True, [(n - 2, n - 1, SWEEP)])


def neutral_to_corral(side, row):
    """Carry a load from the neutral zone straight to the CORRAL for the human player."""
    end = NZ[row][-1]
    pts = ([wp(end[0], end[1], 0.0, None)] + lane_in(side)
           + [wp(2.20, LANE_L, 180.0, 180), wp(1.30, 0.70, 180.0, 180), CORRAL])
    pts[1] = (pts[1][0], pts[1][1], 0.0, pts[1][3])
    pts[2] = (pts[2][0], pts[2][1], 0.0, pts[2][3])
    n = len(pts)
    return leg(f"Neutral {row} to Corral", pts, False, [(n - 2, n - 1, WALL)])


def chute_to_corral():
    end = CHUTE[-1]
    return leg("Chute to Corral", [wp(end[0], end[1], 180.0, -90), CORRAL],
               False, [(0, 1, WALL)])


SHOOT = ("named", "Shoot", False)
FEED = ("named", "FeedCorral", False)

# ---- The 20 autos -------------------------------------------------------------------------------

S, Q = START, SHOT
# Detour that keeps clear of the TOWER (x < 1.15, y 3.25..4.24 on blue) between the two ends.
AROUND_TOWER = [wp(2.15, 4.90, None, -90), wp(2.15, 2.60, None, -90)]
CORRAL_POSE = (CORRAL[0], CORRAL[1], 180.0)
DEPOT_TO_CORRAL = AROUND_TOWER + [wp(1.30, 0.70, 180.0, 180)]

SO = START_OUT

autos = {
    # ---- LOW TRENCH start (outpost side) ----
    "R26 01 Low Trench - Hold, Neutral x2": [
        to_neutral("L Start Out", SO["L"], "L", "LC"), from_neutral("L", "LC", "L Shot", Q["L"]), SHOOT,
        to_neutral("L Shot", Q["L"], "L", "L"), from_neutral("L", "L", "L Shot", Q["L"]), SHOOT],
    "R26 02 Low Trench - Hold, Neutral, Chute": [
        to_neutral("L Start Out", SO["L"], "L", "LX"), from_neutral("L", "LX", "L Shot", Q["L"]), SHOOT,
        to_chute("L Shot", Q["L"]), chute_to("O Shot", Q["O"]), SHOOT],
    "R26 03 Low Trench - Hold, Chute, Neutral to HP": [
        to_chute("L Start", S["L"]), chute_to("O Shot", Q["O"]), SHOOT,
        to_neutral("O Shot", Q["O"], "L", "LC"), neutral_to_corral("L", "LC"), FEED],
    "R26 04 Low Trench - Hold, Chute, Neutral": [
        to_chute("L Start", S["L"]), chute_to("O Shot", Q["O"]), SHOOT,
        to_neutral("O Shot", Q["O"], "L", "LC"), from_neutral("L", "LC", "L Shot", Q["L"]), SHOOT],

    # ---- LOW BUMP start ----
    "R26 05 Low Bump - Hold, Neutral x2": [
        to_neutral("BL Start Out", SO["BL"], "L", "L"), from_neutral("L", "L", "L Shot", Q["L"]), SHOOT,
        to_neutral("L Shot", Q["L"], "L", "L"), from_neutral("L", "L", "L Shot", Q["L"]), SHOOT],
    "R26 06 Low Bump - Hold, Neutral, Chute": [
        to_neutral("BL Start Out", SO["BL"], "L", "LC"), from_neutral("L", "LC", "L Shot", Q["L"]), SHOOT,
        to_chute("L Shot", Q["L"]), chute_to("O Shot", Q["O"]), SHOOT],
    "R26 07 Low Bump - Hold, Chute, Neutral": [
        to_chute("BL Start", S["BL"]), chute_to("O Shot", Q["O"]), SHOOT,
        to_neutral("O Shot", Q["O"], "L", "LC"), from_neutral("L", "LC", "L Shot", Q["L"]), SHOOT],
    "R26 08 Low Bump - Hold, Chute, Neutral to HP": [
        to_chute("BL Start", S["BL"]), chute_to("O Shot", Q["O"]), SHOOT,
        to_neutral("O Shot", Q["O"], "L", "LC"), neutral_to_corral("L", "LC"), FEED],

    # ---- CENTER start (in front of the HUB) ----
    "R26 09 Center - Preload, Depot, Chute": [
        start_to_shot("C"), SHOOT,
        to_depot("C Shot", Q["C"]), depot_to("D Shot", Q["D"], came_from=Q["C"]), SHOOT,
        to_chute("D Shot", Q["D"], via=AROUND_TOWER), chute_to("O Shot", Q["O"]), SHOOT],
    "R26 10 Center - Hold, Chute, Neutral to HP": [
        to_chute("C Start", S["C"]), chute_to("O Shot", Q["O"]), SHOOT,
        to_neutral("O Shot", Q["O"], "L", "LC"), neutral_to_corral("L", "LC"), FEED],
    "R26 11 Center - Hold, Depot, Neutral": [
        to_depot("C Start", S["C"]), depot_to("D Shot", Q["D"], came_from=S["C"]), SHOOT,
        to_neutral("D Shot", Q["D"], "H", "HC"), from_neutral("H", "HC", "H Shot", Q["H"]), SHOOT],
    "R26 12 Center - Hold, Chute, Neutral": [
        to_chute("C Start", S["C"]), chute_to("O Shot", Q["O"]), SHOOT,
        to_neutral("O Shot", Q["O"], "L", "LC"), from_neutral("L", "LC", "L Shot", Q["L"]), SHOOT],

    # ---- HIGH BUMP start ----
    "R26 13 High Bump - Hold, Depot, Neutral": [
        to_depot("BH Start", S["BH"]), depot_to("D Shot", Q["D"], came_from=S["BH"]), SHOOT,
        to_neutral("D Shot", Q["D"], "H", "HC"), from_neutral("H", "HC", "H Shot", Q["H"]), SHOOT],
    "R26 14 High Bump - Hold, Neutral x2": [
        to_neutral("BH Start Out", SO["BH"], "H", "H"), from_neutral("H", "H", "H Shot", Q["H"]), SHOOT,
        to_neutral("H Shot", Q["H"], "H", "H"), from_neutral("H", "H", "H Shot", Q["H"]), SHOOT],
    "R26 15 High Bump - Hold, Neutral, Depot to HP": [
        to_neutral("BH Start Out", SO["BH"], "H", "HC"), from_neutral("H", "HC", "H Shot", Q["H"]), SHOOT,
        to_depot("H Shot", Q["H"]),
        depot_to("Corral", CORRAL_POSE, via=DEPOT_TO_CORRAL, came_from=Q["H"]), FEED],
    "R26 16 High Bump - Hold, Neutral, Depot": [
        to_neutral("BH Start Out", SO["BH"], "H", "HC"), from_neutral("H", "HC", "H Shot", Q["H"]), SHOOT,
        to_depot("H Shot", Q["H"]), depot_to("D Shot", Q["D"], came_from=Q["H"]), SHOOT],

    # ---- HIGH TRENCH start (depot side) ----
    "R26 17 High Trench - Hold, Neutral x2": [
        to_neutral("H Start Out", SO["H"], "H", "HC"), from_neutral("H", "HC", "H Shot", Q["H"]), SHOOT,
        to_neutral("H Shot", Q["H"], "H", "H"), from_neutral("H", "H", "H Shot", Q["H"]), SHOOT],
    "R26 18 High Trench - Hold, Neutral, Depot": [
        to_neutral("H Start Out", SO["H"], "H", "HX"), from_neutral("H", "HX", "H Shot", Q["H"]), SHOOT,
        to_depot("H Shot", Q["H"]), depot_to("D Shot", Q["D"], came_from=Q["H"]), SHOOT],
    "R26 19 High Trench - Hold, Depot, Neutral": [
        to_depot("H Start", S["H"]), depot_to("D Shot", Q["D"], came_from=S["H"]), SHOOT,
        to_neutral("D Shot", Q["D"], "H", "HC"), from_neutral("H", "HC", "H Shot", Q["H"]), SHOOT],
    "R26 20 High Trench - Hold, Neutral, Depot to HP": [
        to_neutral("H Start Out", SO["H"], "H", "HX"), from_neutral("H", "HX", "H Shot", Q["H"]), SHOOT,
        to_depot("H Shot", Q["H"]),
        depot_to("Corral", CORRAL_POSE, via=DEPOT_TO_CORRAL, came_from=Q["H"]), FEED],
}


def step_json(step):
    kind, name, intake = step
    if kind == "named":
        return {"type": "named", "data": {"name": name}}
    if intake:
        return {"type": "deadline", "data": {"commands": [
            {"type": "path", "data": {"pathName": name}},
            {"type": "named", "data": {"name": "IntakeDown"}}]}}
    return {"type": "path", "data": {"pathName": name}}


for auto_name, steps in autos.items():
    commands = [step_json(s) for s in steps] + [{"type": "named", "data": {"name": "StopAll"}}]
    auto = {"version": "2025.0",
            "command": {"type": "sequential", "data": {"commands": commands}},
            "resetOdom": True, "folder": FOLDER, "choreoAuto": False}
    with open(os.path.join(AUTOS, auto_name + ".auto"), "w", encoding="utf-8") as f:
        json.dump(auto, f, indent=2)

print(f"{len(autos)} autos, {len(written)} paths written to the '{FOLDER}' folder")
