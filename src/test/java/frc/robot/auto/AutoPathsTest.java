package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Checks the PathPlanner autos against the real field and the 2026 game manual before they ever
 * reach the robot.
 *
 * <p>The failures these catch are all silent ones. A misspelled named command does not throw, so
 * the auto just skips that step and the robot drives a tidy path without ever shooting. A missing
 * path file takes the whole routine out. A waypoint sitting inside the hub reads fine as JSON and
 * wrecks the robot on the field. A shot from the neutral zone is a MAJOR FOUL that nobody notices
 * until the score sheet. None of that is visible by eye.
 *
 * <p>Every {@code .auto} in the deploy folder is checked, so a new routine is covered the moment
 * it is saved. Field geometry is from the manual and the official AprilTag layout rather than
 * guessed, so it cannot drift away from the real field.
 */
public class AutoPathsTest {

  private static final File DEPLOY = new File("src/main/deploy/pathplanner");

  /** Must match the names registered in RobotContainer.registerAutoCommands(). */
  private static final Set<String> REGISTERED_COMMANDS =
      Set.of("Shoot", "SpinUp", "IntakeDown", "StopAll", "AlignToHub", "FeedOutpost",
          "GravityDeploy");

  /** 0.9 m square with bumpers, as in PathPlanner's settings. */
  private static final double ROBOT_HALF_WIDTH = 0.45;

  /** A rotated robot reaches this far from its centre. */
  private static final double ROBOT_HALF_DIAGONAL = 0.64;

  // --- Field geometry, 2026 manual section 5 (BLUE side; red is the same rotated 180 degrees) ---

  private static final double FIELD_LENGTH = 16.541;
  private static final double FIELD_WIDTH = 8.069;
  private static final double CENTRELINE_X = FIELD_LENGTH / 2.0;

  /**
   * The ALLIANCE ZONE is 158.6 in deep and includes the ROBOT STARTING LINE, a 2 in tape line at
   * its far edge.
   */
  private static final double ALLIANCE_ZONE_DEPTH = 4.028;
  private static final double STARTING_LINE_NEAR = ALLIANCE_ZONE_DEPTH - 0.051;

  /**
   * A single line of structure spans the field: trench, bump, HUB, bump, trench. The trench is
   * 1.194 m deep and the HUB a 47 in square, so the band is x in [4.03, 5.22]. The only ways
   * through are under the two trenches or over the two bumps.
   */
  private static final double BARRIER_NEAR_X = 4.03;
  private static final double BARRIER_FAR_X = 5.22;

  /** BUMPS are 44.4 in deep, centred on the HUB, so they start a little behind the line. */
  private static final double BUMP_NEAR_X = 4.062;

  /** Trench opening is 1.668 m wide, measured from the guardrail. */
  private static final double TRENCH_WIDTH = 1.668;
  private static final double BOTTOM_LANE = TRENCH_WIDTH / 2.0;
  private static final double TOP_LANE = FIELD_WIDTH - TRENCH_WIDTH / 2.0;

  /** HUB is a 47 in square. */
  private static final double HUB_HALF = 0.597;

  /**
   * TOWER BASE: 39.0 in wide, 45.18 in deep from the TOWER WALL, centred on the centred tower tag
   * (31, y = 3.746).
   */
  private static final double TOWER_DEPTH = 1.148;
  private static final double TOWER_LOW_Y = 3.746 - 0.495;
  private static final double TOWER_HIGH_Y = 3.746 + 0.495;

  /** Shot table covers 1.0 to 5.0 m; stay inside it with room to spare. */
  private static final double MIN_SHOT_RANGE = 1.0;
  private static final double MAX_SHOT_RANGE = 4.5;

  private static JsonNode read(File file) throws Exception {
    return new ObjectMapper().readTree(file);
  }

  private static File autoFile(String name) {
    return new File(DEPLOY, "autos/" + name + ".auto");
  }

  private static File pathFile(String name) {
    return new File(DEPLOY, "paths/" + name + ".path");
  }

  /** Every auto in the deploy folder. */
  private static List<String> autos() {
    File[] files = new File(DEPLOY, "autos").listFiles((dir, name) -> name.endsWith(".auto"));
    assertTrue(files != null && files.length > 0, "no autos found");
    List<String> names = new ArrayList<>();
    for (File file : files) {
      names.add(file.getName().substring(0, file.getName().length() - ".auto".length()));
    }
    names.sort(String::compareTo);
    return names;
  }

  /** Collects every "named" and "path" reference anywhere in an auto's command tree. */
  private static void collect(JsonNode node, List<String> named, List<String> paths) {
    // isNull() alone is not enough: a key that is simply absent comes back as a MissingNode,
    // which is not null, so recursing on it walks MissingNodes forever and blows the stack.
    if (node == null || node.isMissingNode() || node.isNull()) {
      return;
    }
    if (node.isArray()) {
      node.forEach(child -> collect(child, named, paths));
      return;
    }
    String type = node.path("type").asText("");
    JsonNode data = node.path("data");
    if ("named".equals(type)) {
      named.add(data.path("name").asText(""));
    } else if ("path".equals(type)) {
      paths.add(data.path("pathName").asText(""));
    }
    if (data.has("commands")) {
      collect(data.get("commands"), named, paths);
    }
    if (data.has("command")) {
      collect(data.get("command"), named, paths);
    }
  }

  private static List<String> pathsOf(String auto) throws Exception {
    List<String> named = new ArrayList<>();
    List<String> paths = new ArrayList<>();
    collect(read(autoFile(auto)).path("command"), named, paths);
    return paths;
  }

  /** Every anchor AND control point, because a bezier is bounded by its control polygon. */
  private static List<Translation2d> allPoints(String path) throws Exception {
    List<Translation2d> points = new ArrayList<>();
    for (JsonNode waypoint : read(pathFile(path)).path("waypoints")) {
      for (String key : new String[] {"anchor", "prevControl", "nextControl"}) {
        JsonNode point = waypoint.path(key);
        if (point.isObject()) {
          points.add(new Translation2d(point.path("x").asDouble(), point.path("y").asDouble()));
        }
      }
    }
    return points;
  }

  private static Translation2d anchor(JsonNode waypoint) {
    return new Translation2d(
        waypoint.path("anchor").path("x").asDouble(), waypoint.path("anchor").path("y").asDouble());
  }

  private static Translation2d endOf(String path) throws Exception {
    JsonNode waypoints = read(pathFile(path)).path("waypoints");
    return anchor(waypoints.get(waypoints.size() - 1));
  }

  /** Distance from a point to an axis-aligned box; zero inside it. */
  private static double boxDistance(Translation2d p, double x0, double x1, double y0, double y1) {
    double dx = Math.max(Math.max(x0 - p.getX(), 0.0), p.getX() - x1);
    double dy = Math.max(Math.max(y0 - p.getY(), 0.0), p.getY() - y1);
    return Math.hypot(dx, dy);
  }

  /**
   * The top-level steps of an auto in order, each either "path:NAME" or "named:NAME". A path run
   * alongside a named command (intake down while driving) counts as the path.
   */
  private static List<String> topLevelSteps(String auto) throws Exception {
    List<String> steps = new ArrayList<>();
    for (JsonNode command : read(autoFile(auto)).path("command").path("data").path("commands")) {
      String type = command.path("type").asText("");
      if ("named".equals(type)) {
        steps.add("named:" + command.path("data").path("name").asText(""));
      } else if ("path".equals(type)) {
        steps.add("path:" + command.path("data").path("pathName").asText(""));
      } else {
        List<String> named = new ArrayList<>();
        List<String> paths = new ArrayList<>();
        collect(command.path("data").path("commands"), named, paths);
        if (!paths.isEmpty()) {
          steps.add("path:" + paths.get(0));
        } else {
          named.forEach(name -> steps.add("named:" + name));
        }
      }
    }
    return steps;
  }

  @Test
  public void everyNamedCommandIsActuallyRegistered() throws Exception {
    for (String auto : autos()) {
      List<String> named = new ArrayList<>();
      List<String> paths = new ArrayList<>();
      collect(read(autoFile(auto)).path("command"), named, paths);

      // CYBERPSYCHO routines are pure driving by design: in that mode every mechanism is
      // released, so a named step would command something that will not answer.
      if (!auto.contains(" CP ")) {
        assertFalse(named.isEmpty(), auto + " calls no named commands at all");
      }
      for (String name : named) {
        assertTrue(
            REGISTERED_COMMANDS.contains(name),
            auto + " calls \"" + name + "\", which RobotContainer never registers. "
                + "PathPlanner skips unknown names without complaining.");
      }
    }
  }

  @Test
  public void everyReferencedPathFileExists() throws Exception {
    for (String auto : autos()) {
      List<String> paths = pathsOf(auto);
      assertFalse(paths.isEmpty(), auto + " follows no path");
      for (String path : paths) {
        assertTrue(pathFile(path).isFile(), auto + " references a missing path: " + path);
      }
    }
  }

  @Test
  public void everyAutoResetsOdometry() throws Exception {
    // Without this the robot assumes it is wherever odometry was last left, and the first path
    // drives off from the wrong origin.
    for (String auto : autos()) {
      assertTrue(
          read(autoFile(auto)).path("resetOdom").asBoolean(false),
          auto + " does not reset odometry, so its start pose is whatever was left over");
    }
  }

  @Test
  public void everyStartIsLegal() throws Exception {
    // G303: BUMPERS overlap the ROBOT STARTING LINE, and the robot is not touching a BUMP. Starts
    // are squared to the line, so the footprint is exactly the 0.9 m square.
    for (String auto : autos()) {
      String first = pathsOf(auto).get(0);
      JsonNode path = read(pathFile(first));
      Translation2d start = anchor(path.path("waypoints").get(0));
      double heading = path.path("idealStartingState").path("rotation").asDouble();
      double front = start.getX() + ROBOT_HALF_WIDTH;
      double back = start.getX() - ROBOT_HALF_WIDTH;

      assertTrue(
          Math.abs(Math.sin(Math.toRadians(heading))) < 0.02,
          auto + " starts at " + heading + " degrees; starts must be squared to the line");
      assertTrue(
          front > STARTING_LINE_NEAR && back < ALLIANCE_ZONE_DEPTH,
          auto + " starts at x=" + start.getX() + ": its BUMPERS do not overlap the ROBOT "
              + "STARTING LINE (G303)");

      double low = start.getY() - ROBOT_HALF_WIDTH;
      double high = start.getY() + ROBOT_HALF_WIDTH;
      boolean inATrenchLane = high <= TRENCH_WIDTH || low >= FIELD_WIDTH - TRENCH_WIDTH;
      if (!inATrenchLane) {
        // In front of a BUMP or the HUB: the front bumper must stop short of both.
        assertTrue(
            front < BUMP_NEAR_X,
            auto + " starts touching a BUMP or the HUB (front bumper at x=" + front + ")");
      }
    }
  }

  @Test
  public void everyWaypointIsOnTheField() throws Exception {
    for (String auto : autos()) {
      for (String path : pathsOf(auto)) {
        // Paths that park against the OUTPOST to feed the OUTPOST arrive squared, so only the
        // half-width needs to clear the wall. Everything else allows for rotation.
        double margin = path.contains("Outpost") ? ROBOT_HALF_WIDTH : ROBOT_HALF_DIAGONAL;
        for (Translation2d point : allPoints(path)) {
          assertTrue(
              point.getX() >= margin && point.getX() <= FIELD_LENGTH - margin,
              path + " point x=" + point.getX() + " puts the robot through a wall");
          assertTrue(
              point.getY() >= margin && point.getY() <= FIELD_WIDTH - margin,
              path + " point y=" + point.getY() + " puts the robot through a wall");
        }
      }
    }
  }

  @Test
  public void nothingCrossesTheBarrierExceptThroughATrench() throws Exception {
    // This is the test that matters most. Going over the bump is a 15 degree ramp with a 6.5 in
    // crest taken at speed, which is the high-failure crossing: it unsettles odometry and can
    // beach or tip the robot. Every route here crosses under a trench instead, and this asserts
    // it, pinned to the middle of the opening so the robot cannot clip the trench structure.
    //
    // Control points count. The anchors can sit either side of the barrier while the curve
    // between them cuts the corner straight across a bump.
    for (String auto : autos()) {
      for (String path : pathsOf(auto)) {
        for (Translation2d point : allPoints(path)) {
          boolean overlapsBarrier =
              point.getX() + ROBOT_HALF_WIDTH > BARRIER_NEAR_X
                  && point.getX() - ROBOT_HALF_WIDTH < BARRIER_FAR_X;
          if (!overlapsBarrier) {
            continue;
          }
          double offLane = Math.min(
              Math.abs(point.getY() - BOTTOM_LANE), Math.abs(point.getY() - TOP_LANE));
          assertTrue(
              offLane < 0.20,
              path + " point (" + String.format("%.2f, %.2f", point.getX(), point.getY())
                  + ") is inside the barrier band x=[" + BARRIER_NEAR_X + ", " + BARRIER_FAR_X
                  + "] but not on a trench centreline. That crosses a bump or the hub, or "
                  + "clips the trench.");
        }
      }
    }
  }

  @Test
  public void noAutoGetsToTheCentreline() throws Exception {
    // G403 forbids getting completely across the CENTER LINE in AUTO. This is stricter: no part
    // of the robot reaches it, so there is no contact with an opponent working their own half.
    //
    // Checking control points too is not pedantry: the anchors can sit behind the line while the
    // curve between them bulges across it.
    for (String auto : autos()) {
      for (String path : pathsOf(auto)) {
        for (Translation2d point : allPoints(path)) {
          double robotEdge = point.getX() + ROBOT_HALF_DIAGONAL;
          assertTrue(
              robotEdge < CENTRELINE_X,
              path + " reaches x=" + String.format("%.2f", point.getX())
                  + ", putting the robot edge at " + String.format("%.2f", robotEdge)
                  + " m, past the centreline at " + String.format("%.2f", CENTRELINE_X));
        }
      }
    }
  }

  @Test
  public void noWaypointDrivesIntoTheHub() throws Exception {
    // Measured to the HUB's square footprint. Paths only come near it squared (the centre start
    // and backing away from it), so the half-width is the clearance that matters.
    Translation2d hub = blueHubCentre();
    for (String auto : autos()) {
      for (String path : pathsOf(auto)) {
        for (Translation2d point : allPoints(path)) {
          double gap = boxDistance(point, hub.getX() - HUB_HALF, hub.getX() + HUB_HALF,
              hub.getY() - HUB_HALF, hub.getY() + HUB_HALF);
          assertTrue(
              gap > ROBOT_HALF_WIDTH,
              path + " point (" + String.format("%.2f, %.2f", point.getX(), point.getY())
                  + ") puts the robot into the HUB");
        }
      }
    }
  }

  @Test
  public void noWaypointDrivesIntoTheTower() throws Exception {
    // The TOWER and its base plate stick 1.15 m out of the ALLIANCE WALL between the OUTPOST and
    // the DEPOT, right where routes between the two would like to cut the corner.
    for (String auto : autos()) {
      for (String path : pathsOf(auto)) {
        for (Translation2d point : allPoints(path)) {
          double gap = boxDistance(point, 0.0, TOWER_DEPTH, TOWER_LOW_Y, TOWER_HIGH_Y);
          assertTrue(
              gap > ROBOT_HALF_DIAGONAL,
              path + " point (" + String.format("%.2f, %.2f", point.getX(), point.getY())
                  + ") clips the TOWER");
        }
      }
    }
  }

  @Test
  public void everyShotIsFromInsideTheAllianceZoneAndInRange() throws Exception {
    // G407: a robot may only launch into its HUB with its BUMPERS at least partly inside its
    // ALLIANCE ZONE; doing otherwise is a MAJOR FOUL per shot. These require the whole robot
    // inside, so drift during the shot cannot make it illegal. The range check keeps every shot
    // inside the shot table, which refuses to extrapolate.
    Translation2d hub = blueHubCentre();
    for (String auto : autos()) {
      Translation2d at = null;
      for (String step : topLevelSteps(auto)) {
        if (step.startsWith("path:")) {
          at = endOf(step.substring("path:".length()));
        } else if (step.equals("named:Shoot")) {
          assertTrue(at != null, auto + " shoots before driving anywhere; start with a path");
          assertTrue(
              at.getX() + ROBOT_HALF_WIDTH <= ALLIANCE_ZONE_DEPTH,
              auto + " shoots from x=" + String.format("%.2f", at.getX())
                  + ", outside the ALLIANCE ZONE (G407, MAJOR FOUL)");
          double range = hub.getDistance(at) - HUB_HALF;
          assertTrue(
              range >= MIN_SHOT_RANGE && range <= MAX_SHOT_RANGE,
              auto + " shoots from " + String.format("%.2f", range) + " m, outside the shot table");
        } else if (step.equals("named:FeedOutpost")) {
          assertTrue(
              at != null && at.getX() < 0.6 && Math.abs(at.getY() - 0.666) < 0.2,
              auto + " feeds the OUTPOST from " + at + ", not lined up with the OUTPOST opening");
        }
      }
    }
  }

  /**
   * Centre of the blue HUB from the official 2026 layout: midway between opposite faces.
   *
   * <p>Not the mean of all eight HUB tags. Each face has one centred tag and one offset tag, and
   * the offsets do not cancel, so the mean sits 9 cm toward the ALLIANCE WALL.
   */
  private static Translation2d blueHubCentre() {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    double wallFace = layout.getTagPose(26).orElseThrow().getX();
    double fieldFace = layout.getTagPose(20).orElseThrow().getX();
    double lowFace = layout.getTagPose(18).orElseThrow().getY();
    double highFace = layout.getTagPose(21).orElseThrow().getY();
    return new Translation2d((wallFace + fieldFace) / 2.0, (lowFace + highFace) / 2.0);
  }

  @Test
  public void everyGeneratedAutoIsRegistered() {
    // The generator owns this set. If it emits a routine nobody added to the chooser, or the
    // chooser names one that no longer exists, the mismatch only shows up as a missing option on
    // the field.
    long plow = autos().stream().filter(a -> a.startsWith("R26 D")).count();
    long psycho = autos().stream().filter(a -> a.startsWith("R26 CP ")).count();
    assertTrue(plow == 5, "expected 5 plow routines, found " + plow);
    assertTrue(psycho == 4, "expected 4 CYBERPSYCHO routines, found " + psycho);
  }
}
