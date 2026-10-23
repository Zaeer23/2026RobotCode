package frc.robot.auto;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Generates the REBUILT autonomous routines as PathPlanner .path and .auto files.
 *
 * <p>Run it with {@code ./gradlew generateAutos}. Re-running replaces every file this generator
 * owns, so make changes here — a pose, a speed, a routine — rather than in the PathPlanner GUI, or
 * the GUI edits are overwritten the next time anyone runs it. Paths and autos outside this
 * generator's folder are never touched.
 *
 * <p>This is a port of tools/generate_rebuilt_autos.py, which produced the previous twenty
 * routines. The port exists because that script cannot run here: the only python on this machine
 * is the Microsoft Store stub, which prints an install advert and exits. A generator nobody on the
 * team can run is worse than no generator, because the files it owns drift from the source that
 * claims to own them. This runs on the JVM the robot project already needs.
 *
 * <p>All coordinates are BLUE-side field metres, WPILib convention with the blue wall at x=0.
 * PathPlanner mirrors them for red at runtime. Geometry comes from the 2026 manual and the
 * official AprilTag layout: ROBOT STARTING LINE far edge 4.028 m from the wall, BUMPS start at
 * 4.062 m, HUB centre (4.626, 4.035), TRENCH lanes centred on y=0.834 and y=7.235, OUTPOST opening
 * centred on tag 29 at y=0.666, TOWER base x 0..1.148 and y 3.25..4.24, DEPOT x 0..0.686 and
 * y 5.40..6.47, neutral FUEL starting at x 7.36.
 *
 * <p>After regenerating, run {@code ./gradlew test}: AutoPathsTest checks the routes against the
 * game manual and AutoCatalogTest checks each routine still fits the 20 s AUTO period.
 */
public final class RebuiltAutoGenerator {

  private RebuiltAutoGenerator() {}

  /** Every file carrying this folder tag is deleted and rewritten on each run. */
  private static final String FOLDER = "Rebuilt HP";

  private static final double[] HUB = {4.626, 4.035};
  private static final double LANE_L = 0.834;
  private static final double LANE_H = 8.069 - 0.834;

  private static final double TRANSIT = 4.2;  // m/s on open carpet (77% of the 5.45 top speed)
  private static final double ACCEL = 3.2;    // m/s^2
  private static final double SWEEP = 1.5;    // m/s while intaking a row of FUEL
  private static final double WALL = 1.0;     // m/s for the last stretch into a wall

  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final Map<String, Boolean> written = new LinkedHashMap<>();

  private static File pathsDir;
  private static File autosDir;

  // ---- Waypoints ---------------------------------------------------------------------------

  /**
   * One waypoint.
   *
   * @param heading robot heading held here, or null to carry the previous one
   * @param tangent direction of travel through the point in degrees, or null to infer it from the
   *                neighbours. Setting it is how a path is told to leave straight down a trench
   *                lane instead of cutting the corner into the structure.
   */
  private record Wp(double x, double y, Double heading, Double tangent) {}

  private static Wp wp(double x, double y, Double h, Double t) {
    return new Wp(x, y, h, t);
  }

  private static double wrap(double deg) {
    return (deg + 180.0) % 360.0 - 180.0;
  }

  /** Heading that points the robot's BACK (turret and camera) at the HUB. */
  private static double backToHub(double x, double y) {
    return Math.round((Math.toDegrees(Math.atan2(HUB[1] - y, HUB[0] - x)) - 180.0) * 10.0) / 10.0;
  }

  private static Wp shot(double x, double y) {
    return wp(x, y, wrap(backToHub(x, y)), null);
  }

  private static final Map<String, Wp> START = new LinkedHashMap<>();
  private static final Map<String, Wp> SHOT = new LinkedHashMap<>();

  static {
    START.put("L", wp(3.57, LANE_L, 180.0, null));
    START.put("BL", wp(3.57, 2.55, 180.0, null));
    START.put("C", wp(3.55, 4.035, 180.0, null));
    START.put("BH", wp(3.57, 5.52, 180.0, null));
    START.put("H", wp(3.57, LANE_H, 180.0, null));

    SHOT.put("L", shot(2.90, LANE_L));
    SHOT.put("H", shot(2.90, LANE_H));
    SHOT.put("BL", shot(2.75, 2.45));
    SHOT.put("BH", shot(2.75, 5.62));
    SHOT.put("C", wp(2.30, 4.035, 180.0, null));
    SHOT.put("O", shot(1.55, 1.40));
    SHOT.put("D", shot(1.90, 6.37));
  }

  private static List<Wp> laneOut(String side) {
    double y = "L".equals(side) ? LANE_L : LANE_H;
    return List.of(wp(3.55, y, null, 0.0), wp(5.95, y, null, 0.0));
  }

  private static List<Wp> laneIn(String side) {
    double y = "L".equals(side) ? LANE_L : LANE_H;
    return List.of(wp(5.95, y, null, 180.0), wp(3.55, y, null, 180.0));
  }

  /**
   * Neutral-zone sweep rows, driven along x=7.05 with the intake facing the FUEL.
   *
   * <p>"HO" is the one the human-player strategy uses: a single full-height sweep from the high
   * side down to the low side. It collects the middle of the neutral zone and leaves the robot at
   * the bottom, lined up with the low trench lane and therefore with the OUTPOST, instead of
   * collecting a short row and then crossing the field empty.
   */
  private static List<Wp> neutralRow(String row) {
    switch (row) {
      case "L": return List.of(wp(7.05, 1.70, 0.0, 90.0), wp(7.05, 2.70, 0.0, 90.0));
      case "LC": return List.of(wp(7.05, 1.70, 0.0, 90.0), wp(7.05, 3.20, 0.0, 90.0));
      case "H": return List.of(wp(7.05, 6.37, 0.0, -90.0), wp(7.05, 5.37, 0.0, -90.0));
      case "HC": return List.of(wp(7.05, 6.37, 0.0, -90.0), wp(7.05, 4.87, 0.0, -90.0));
      case "HO": return List.of(wp(7.05, 6.37, 0.0, -90.0), wp(7.05, 1.70, 0.0, -90.0));
      // Sweeps downward from mid-field, for routines that go out through the LOW lane. Ends low,
      // already lined up with the OUTPOST, instead of finishing high and crossing back.
      case "LD": return List.of(wp(7.05, 3.80, 0.0, -90.0), wp(7.05, 1.70, 0.0, -90.0));
      default: throw new IllegalArgumentException("unknown neutral row " + row);
    }
  }

  private static final Wp OUTPOST = wp(0.50, 0.666, 180.0, 180.0);

  // Coming from below the robot swings wide of the TOWER and sweeps the DEPOT upward; from above
  // it sweeps downward. Either way the row covers the DEPOT's full 42 in.
  private static final List<Wp> DEPOT_UP = List.of(
      wp(1.85, 4.95, 180.0, 135.0), wp(1.20, 5.30, 180.0, 90.0), wp(1.20, 6.75, 180.0, 90.0));
  private static final List<Wp> DEPOT_DOWN = List.of(
      wp(1.55, 6.95, 180.0, 180.0), wp(1.20, 6.55, 180.0, -90.0), wp(1.20, 5.30, 180.0, -90.0));

  private static List<Wp> depotRow(Wp origin) {
    return origin.y() > 6.0 ? DEPOT_DOWN : DEPOT_UP;
  }

  // ---- Path writing ------------------------------------------------------------------------

  private static double tangentOf(List<Wp> pts, int i) {
    Wp p = pts.get(i);
    if (p.tangent() != null) {
      return Math.toRadians(p.tangent());
    }
    if (i == 0) {
      Wp n = pts.get(1);
      return Math.atan2(n.y() - p.y(), n.x() - p.x());
    }
    if (i == pts.size() - 1) {
      Wp q = pts.get(i - 1);
      return Math.atan2(p.y() - q.y(), p.x() - q.x());
    }
    Wp q = pts.get(i - 1);
    Wp n = pts.get(i + 1);
    double a = Math.atan2(p.y() - q.y(), p.x() - q.x());
    double b = Math.atan2(n.y() - p.y(), n.x() - p.x());
    return Math.atan2(Math.sin(a) + Math.sin(b), Math.cos(a) + Math.cos(b));
  }

  private static double round4(double v) {
    return Math.round(v * 10000.0) / 10000.0;
  }

  private record Slow(int from, int to, double speed) {}

  /** Writes one .path, skipping it if a leg with this name was already emitted this run. */
  private static String writePath(String name, List<Wp> pts, List<Slow> slow) throws IOException {
    if (written.containsKey(name)) {
      return name;
    }

    ArrayNode waypoints = MAPPER.createArrayNode();
    for (int i = 0; i < pts.size(); i++) {
      Wp p = pts.get(i);
      double theta = tangentOf(pts, i);
      ObjectNode entry = MAPPER.createObjectNode();
      ObjectNode anchor = entry.putObject("anchor");
      anchor.put("x", round4(p.x()));
      anchor.put("y", round4(p.y()));

      if (i > 0) {
        Wp q = pts.get(i - 1);
        double d = 0.38 * Math.hypot(p.x() - q.x(), p.y() - q.y());
        ObjectNode prev = entry.putObject("prevControl");
        prev.put("x", round4(p.x() - d * Math.cos(theta)));
        prev.put("y", round4(p.y() - d * Math.sin(theta)));
      } else {
        entry.putNull("prevControl");
      }

      if (i < pts.size() - 1) {
        Wp n = pts.get(i + 1);
        double d = 0.38 * Math.hypot(n.x() - p.x(), n.y() - p.y());
        ObjectNode next = entry.putObject("nextControl");
        next.put("x", round4(p.x() + d * Math.cos(theta)));
        next.put("y", round4(p.y() + d * Math.sin(theta)));
      } else {
        entry.putNull("nextControl");
      }

      entry.put("isLocked", false);
      entry.putNull("linkedName");
      waypoints.add(entry);
    }

    ArrayNode rotations = MAPPER.createArrayNode();
    for (int i = 1; i < pts.size() - 1; i++) {
      if (pts.get(i).heading() != null) {
        ObjectNode r = MAPPER.createObjectNode();
        r.put("waypointRelativePos", i);
        r.put("rotationDegrees", pts.get(i).heading());
        rotations.add(r);
      }
    }

    ArrayNode zones = MAPPER.createArrayNode();
    for (Slow s : slow) {
      ObjectNode z = MAPPER.createObjectNode();
      z.put("name", "Slow");
      z.put("minWaypointRelativePos", s.from());
      z.put("maxWaypointRelativePos", s.to());
      constraints(z.putObject("constraints"), s.speed());
      zones.add(z);
    }

    double startH = pts.get(0).heading() != null ? pts.get(0).heading() : 0.0;
    double endH = pts.get(pts.size() - 1).heading() != null
        ? pts.get(pts.size() - 1).heading() : startH;

    ObjectNode path = MAPPER.createObjectNode();
    path.put("version", "2025.0");
    path.set("waypoints", waypoints);
    path.set("rotationTargets", rotations);
    path.set("constraintZones", zones);
    path.set("pointTowardsZones", MAPPER.createArrayNode());
    path.set("eventMarkers", MAPPER.createArrayNode());
    constraints(path.putObject("globalConstraints"), TRANSIT);
    ObjectNode goal = path.putObject("goalEndState");
    goal.put("velocity", 0);
    goal.put("rotation", endH);
    path.put("reversed", false);
    path.put("folder", FOLDER);
    ObjectNode ideal = path.putObject("idealStartingState");
    ideal.put("velocity", 0);
    ideal.put("rotation", startH);
    path.put("useDefaultConstraints", false);

    MAPPER.writerWithDefaultPrettyPrinter()
        .writeValue(new File(pathsDir, name + ".path"), path);
    written.put(name, true);
    return name;
  }

  private static void constraints(ObjectNode node, double maxVelocity) {
    node.put("maxVelocity", maxVelocity);
    node.put("maxAcceleration", ACCEL);
    node.put("maxAngularVelocity", 540.0);
    node.put("maxAngularAcceleration", 1000.0);
    node.put("nominalVoltage", 12.0);
    node.put("unlimited", false);
  }

  // ---- Legs --------------------------------------------------------------------------------

  /** One step of a routine: either a path (optionally intaking while it drives) or a named command. */
  private record Step(String kind, String name, boolean intake) {}

  private static final Step SHOOT = new Step("named", "Shoot", false);
  private static final Step FEED = new Step("named", "FeedOutpost", false);

  private static Step leg(String name, List<Wp> pts, boolean intake, List<Slow> slow)
      throws IOException {
    return new Step("path", writePath(name, pts, slow), intake);
  }

  /** Drive to the DEPOT and sweep its full length, intaking. */
  private static Step toDepot(String originName, Wp origin) throws IOException {
    List<Wp> pts = new ArrayList<>();
    pts.add(origin);
    pts.addAll(depotRow(origin));
    return leg(originName + " to Depot", pts, true,
        List.of(new Slow(pts.size() - 2, pts.size() - 1, SWEEP)));
  }

  /** Leave the DEPOT from whichever end the sweep finished on. */
  private static Step depotTo(String destName, Wp dest, Wp cameFrom) throws IOException {
    List<Wp> row = depotRow(cameFrom);
    Wp end = row.get(row.size() - 1);
    String direction = end.y() > 6.0 ? "Up" : "Down";
    List<Wp> pts = List.of(wp(end.x(), end.y(), 180.0, null), dest);
    return leg("Depot " + direction + " to " + destName, pts, false, List.of());
  }

  /** From the alliance side, under the trench, then sweep a row of neutral FUEL. */
  private static Step toNeutral(String originName, Wp origin, String side, String row)
      throws IOException {
    List<Wp> pts = new ArrayList<>();
    pts.add(origin);
    pts.addAll(laneOut(side));
    pts.addAll(neutralRow(row));
    // Square to the lane before the trench; the sweep rows carry their own headings.
    for (int i = 1; i < pts.size() - 2; i++) {
      Wp p = pts.get(i);
      pts.set(i, wp(p.x(), p.y(), 0.0, p.tangent()));
    }
    return leg(originName + " to Neutral " + row, pts, true,
        List.of(new Slow(pts.size() - 2, pts.size() - 1, SWEEP)));
  }

  /**
   * Carry a load from the neutral zone back under the trench to the OUTPOST.
   *
   * <p>Not an intaking leg. The robot is full by this point and the job is delivery, but anything
   * loose in front of it gets pushed along the lane toward the OUTPOST on the way, which is the
   * point of sweeping down to the low side first.
   */
  private static Step neutralToOutpost(String side, String row) throws IOException {
    List<Wp> rowPts = neutralRow(row);
    Wp end = rowPts.get(rowPts.size() - 1);
    List<Wp> pts = new ArrayList<>();
    pts.add(wp(end.x(), end.y(), 0.0, null));
    for (Wp p : laneIn(side)) {
      pts.add(wp(p.x(), p.y(), 0.0, p.tangent()));
    }
    pts.add(wp(2.20, LANE_L, 180.0, 180.0));
    pts.add(wp(1.30, 0.70, 180.0, 180.0));
    pts.add(OUTPOST);
    return leg("Neutral " + row + " to Outpost", pts, false,
        List.of(new Slow(pts.size() - 2, pts.size() - 1, WALL)));
  }

  // ---- Routines ----------------------------------------------------------------------------

  /**
   * The human-player strategy, from one starting spot.
   *
   * <p>Collect the DEPOT, shoot it, cross under the trench, sweep the middle of the neutral zone
   * from the high side down to the low side, then deliver everything still aboard through the
   * OUTPOST opening to the HUMAN PLAYER.
   *
   * <p>The delivery is FeedOutpost, not a shot. The OUTPOST opening is 5.33 m from the hub centre
   * and the flywheel runs out at about 5.29 m (see ShotModel), so a shot from there cannot be
   * made — the FUEL goes to the HUMAN PLAYER instead, which is the point of the strategy.
   */
  private static List<Step> humanPlayerRun(String startKey) throws IOException {
    Wp start = START.get(startKey);
    List<Wp> row = depotRow(start);
    Wp depotEnd = row.get(row.size() - 1);

    List<Step> steps = new ArrayList<>();
    steps.add(toDepot(startKey + " Start", start));

    // Shoot from where the DEPOT sweep ends, without driving to a separate shooting pose first.
    // That pose cost a leg out and a leg back for nothing: the end of the row is 4.4 m from the
    // hub centre, inside the shot model's reach, and at the row's heading of 180 the hub sits
    // about 142 degrees round in the robot frame, well inside the turret's rear travel. Driving
    // to "D Shot" and back was the difference between this routine fitting AUTO and not.
    steps.add(SHOOT);

    // Start the next leg from the end of the row with no carried tangent. The sweep finishes
    // travelling downward, and inheriting that sent the first control point down to y=4.14,
    // straight through the TOWER. Leaving it null lets it aim at the lane instead.
    steps.add(toNeutral("Depot", wp(depotEnd.x(), depotEnd.y(), 180.0, null), "H", "HO"));
    steps.add(neutralToOutpost("L", "HO"));
    steps.add(FEED);
    return steps;
  }

  /**
   * Leaves the OUTPOST pointing back out at the field, so TELEOP starts in position.
   *
   * <p>Finishing parked in the corner facing the wall wastes the first seconds of TELEOP driving
   * back out of it.
   */
  private static Step outpostExit() throws IOException {
    List<Wp> pts = List.of(
        wp(OUTPOST.x(), OUTPOST.y(), 180.0, 0.0),
        wp(2.20, LANE_L, 180.0, 0.0),
        wp(3.55, LANE_L, 0.0, 0.0));
    return leg("Outpost to Lane", pts, false, List.of());
  }

  /**
   * The short version: skip the DEPOT, go straight out for the neutral zone.
   *
   * <p>Same delivery, far less driving. Worth having because the full run is close to the 20 s
   * limit, and a routine that gets cut off mid-delivery hands the HUMAN PLAYER nothing.
   */
  private static List<Step> directRun(String startKey) throws IOException {
    Wp start = START.get(startKey);
    Wp shotPose = SHOT.get(startKey);
    List<Step> steps = new ArrayList<>();

    // Roll off the line to a shooting pose and fire the preload before going anywhere. A routine
    // has to open with a path: shooting from the starting spot itself means the robot never
    // declares where it is, so the preview has no start pose and odometry is never reset against
    // a known point.
    steps.add(leg(startKey + " Start to Shot", List.of(start, shotPose), false, List.of()));
    steps.add(SHOOT);

    // Go out through whichever lane is nearer the start. Sending a centre start all the way up to
    // the high lane and back down cost 21.2 s on its own, past the whole AUTO period.
    boolean highSide = start.y() > 5.0;
    String side = highSide ? "H" : "L";
    String row = highSide ? "HO" : "LD";

    steps.add(toNeutral(startKey + " Shot", shotPose, side, row));
    steps.add(neutralToOutpost("L", row));
    steps.add(FEED);
    steps.add(outpostExit());
    return steps;
  }

  private static Map<String, List<Step>> routines() throws IOException {
    Map<String, List<Step>> autos = new LinkedHashMap<>();
    autos.put("R26 HP 1 High Trench - Depot, Shoot, Neutral to HP", humanPlayerRun("H"));
    autos.put("R26 HP 2 High Bump - Depot, Shoot, Neutral to HP", humanPlayerRun("BH"));
    autos.put("R26 HP 3 Center - Depot, Shoot, Neutral to HP", humanPlayerRun("C"));
    autos.put("R26 HP 4 High Trench - Straight to Neutral to HP", directRun("H"));
    autos.put("R26 HP 5 Center - Straight to Neutral to HP", directRun("C"));
    return autos;
  }

  // ---- Emission ----------------------------------------------------------------------------

  private static ObjectNode stepJson(Step step) {
    ObjectNode node = MAPPER.createObjectNode();
    if ("named".equals(step.kind())) {
      node.put("type", "named");
      node.putObject("data").put("name", step.name());
      return node;
    }
    if (step.intake()) {
      node.put("type", "deadline");
      ArrayNode commands = node.putObject("data").putArray("commands");
      ObjectNode path = commands.addObject();
      path.put("type", "path");
      path.putObject("data").put("pathName", step.name());
      ObjectNode intake = commands.addObject();
      intake.put("type", "named");
      intake.putObject("data").put("name", "IntakeDown");
      return node;
    }
    node.put("type", "path");
    node.putObject("data").put("pathName", step.name());
    return node;
  }

  private static void writeAuto(String name, List<Step> steps) throws IOException {
    ObjectNode auto = MAPPER.createObjectNode();
    auto.put("version", "2025.0");
    ObjectNode command = auto.putObject("command");
    command.put("type", "sequential");
    ArrayNode commands = command.putObject("data").putArray("commands");
    for (Step step : steps) {
      commands.add(stepJson(step));
    }
    ObjectNode stop = commands.addObject();
    stop.put("type", "named");
    stop.putObject("data").put("name", "StopAll");
    auto.put("resetOdom", true);
    auto.put("folder", FOLDER);
    auto.put("choreoAuto", false);
    MAPPER.writerWithDefaultPrettyPrinter().writeValue(new File(autosDir, name + ".auto"), auto);
  }

  /** Removes this generator's previous output, so renamed or dropped legs do not linger. */
  private static void clearFolder(File dir, String extension) throws IOException {
    File[] files = dir.listFiles((d, n) -> n.endsWith(extension));
    if (files == null) {
      return;
    }
    for (File file : files) {
      try {
        var node = MAPPER.readTree(file);
        String folder = node.path("folder").asText("");
        // Also sweeps the previous generator's output, which this run replaces.
        if (FOLDER.equals(folder) || "Rebuilt 20".equals(folder)) {
          Files.delete(file.toPath());
        }
      } catch (IOException ignored) {
        // Not ours to delete if it will not parse.
      }
    }
  }

  public static void main(String[] args) throws IOException {
    File root = new File(args.length > 0 ? args[0] : ".");
    pathsDir = new File(root, "src/main/deploy/pathplanner/paths");
    autosDir = new File(root, "src/main/deploy/pathplanner/autos");
    if (!pathsDir.isDirectory() || !autosDir.isDirectory()) {
      throw new IOException("Run from the project root; no pathplanner folders under " + root);
    }

    clearFolder(autosDir, ".auto");
    clearFolder(pathsDir, ".path");

    Map<String, List<Step>> autos = routines();
    for (Map.Entry<String, List<Step>> entry : autos.entrySet()) {
      writeAuto(entry.getKey(), entry.getValue());
    }

    System.out.println("Wrote " + autos.size() + " autos and " + written.size()
        + " paths into folder \"" + FOLDER + "\".");
    autos.keySet().forEach(name -> System.out.println("  " + name));
    System.out.println("\nNow run: ./gradlew test");
  }
}
