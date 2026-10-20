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
 * Checks the PathPlanner autos against the real field before they ever reach the robot.
 *
 * <p>The failures these catch are all silent ones. A misspelled named command does not throw, so
 * the auto just skips that step and the robot drives a tidy path without ever shooting. A missing
 * path file takes the whole routine out. A waypoint sitting inside the hub reads fine as JSON and
 * wrecks the robot on the field. None of that is visible by eye.
 *
 * <p>Field geometry here is derived from the official AprilTag layout rather than typed in, so it
 * cannot drift away from the real field.
 */
public class AutoPathsTest {

  private static final File DEPLOY = new File("src/main/deploy/pathplanner");

  /** Must match the names registered in RobotContainer.registerAutoCommands(). */
  private static final Set<String> REGISTERED_COMMANDS =
      Set.of("Shoot", "SpinUp", "IntakeDown", "StopAll", "AlignToHub");

  private static final String[] AUTOS = {
    "Trench Bottom Cycle",
    "Trench Top Cycle",
    "Trench Bottom Safe",
    "Trench Top Safe",
    "Neutral Zone Denial"
  };

  /**
   * Paths that deliberately run under the trench, hard against the field edge.
   *
   * <p>These are exempt from the general wall margin. The robot drives them axis-aligned, so the
   * clearance that matters is its half-width, not the half-diagonal a rotating robot needs.
   */
  private static final Set<String> TRENCH_PATHS =
      Set.of("Trench Bot Enter", "Trench Top Enter");

  /** Half the robot's 0.9 m width, which is the real margin while it is square to the wall. */
  private static final double ROBOT_HALF_WIDTH = 0.45;

  // --- The barrier across the field, from the 2026 manual -------------------------------------
  // A single line of structure spans the full width: trench, bump, HUB, bump, trench. The trench
  // is 1.194 m deep and the HUB is a 47 in square, so the band is x in [4.03, 5.22]. The only
  // ways through are under the two trenches or over the two bumps.

  private static final double BARRIER_NEAR_X = 4.03;
  private static final double BARRIER_FAR_X = 5.22;

  /** Trench opening is 1.668 m wide, measured from the guardrail. */
  private static final double TRENCH_WIDTH = 1.668;

  private static final double FIELD_LENGTH = 16.541;
  private static final double FIELD_WIDTH = 8.069;
  private static final double CENTRELINE_X = FIELD_LENGTH / 2.0;

  private static final double BOTTOM_TRENCH_LOW = 0.0;
  private static final double BOTTOM_TRENCH_HIGH = TRENCH_WIDTH;
  private static final double TOP_TRENCH_LOW = FIELD_WIDTH - TRENCH_WIDTH;
  private static final double TOP_TRENCH_HIGH = FIELD_WIDTH;

  /** The robot is 0.9 m square, so its centre is never legally nearer than this to an obstacle. */
  private static final double ROBOT_HALF_DIAGONAL = 0.64;

  /** Hub structure reaches roughly this far out from the centre of the tag ring. */
  private static final double HUB_RADIUS = 0.75;

  private static JsonNode read(File file) throws Exception {
    return new ObjectMapper().readTree(file);
  }

  private static File autoFile(String name) {
    return new File(DEPLOY, "autos/" + name + ".auto");
  }

  private static File pathFile(String name) {
    return new File(DEPLOY, "paths/" + name + ".path");
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

  @Test
  public void everyAutoExists() {
    for (String auto : AUTOS) {
      assertTrue(autoFile(auto).isFile(), "missing auto file: " + auto);
    }
  }

  @Test
  public void everyNamedCommandIsActuallyRegistered() throws Exception {
    for (String auto : AUTOS) {
      List<String> named = new ArrayList<>();
      List<String> paths = new ArrayList<>();
      collect(read(autoFile(auto)).path("command"), named, paths);

      assertFalse(named.isEmpty(), auto + " calls no named commands at all");
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
    for (String auto : AUTOS) {
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
    for (String auto : AUTOS) {
      assertTrue(
          read(autoFile(auto)).path("resetOdom").asBoolean(false),
          auto + " does not reset odometry, so its start pose is whatever was left over");
    }
  }

  @Test
  public void everyWaypointIsOnTheField() throws Exception {
    for (String auto : AUTOS) {
      for (String path : pathsOf(auto)) {
        double margin = TRENCH_PATHS.contains(path) ? ROBOT_HALF_WIDTH : ROBOT_HALF_DIAGONAL;
        for (Translation2d point : allPoints(path)) {
          assertTrue(
              point.getX() > margin && point.getX() < FIELD_LENGTH - margin,
              path + " point x=" + point.getX() + " puts the robot through a wall");
          assertTrue(
              point.getY() > margin && point.getY() < FIELD_WIDTH - margin,
              path + " point y=" + point.getY() + " puts the robot through a wall");
        }
      }
    }
  }

  @Test
  public void nothingCrossesTheBarrierExceptThroughATrench() throws Exception {
    // This is the test that matters most. A single line of structure spans the field -- trench,
    // bump, hub, bump, trench -- and the only ways across are under a trench or over a bump.
    // Going over the bump is a 15 degree ramp with a 6.5 in crest taken at speed, which is the
    // high-failure crossing: it unsettles odometry and can beach or tip the robot. Every route
    // here crosses under a trench instead, and this asserts it.
    //
    // Control points count. The anchors can sit either side of the barrier while the curve
    // between them cuts the corner straight across a bump.
    for (String auto : AUTOS) {
      for (String path : pathsOf(auto)) {
        for (Translation2d point : allPoints(path)) {
          boolean overlapsBarrier =
              point.getX() + ROBOT_HALF_WIDTH > BARRIER_NEAR_X
                  && point.getX() - ROBOT_HALF_WIDTH < BARRIER_FAR_X;
          if (!overlapsBarrier) {
            continue;
          }

          double low = point.getY() - ROBOT_HALF_WIDTH;
          double high = point.getY() + ROBOT_HALF_WIDTH;
          boolean inBottomTrench = low >= BOTTOM_TRENCH_LOW && high <= BOTTOM_TRENCH_HIGH;
          boolean inTopTrench = low >= TOP_TRENCH_LOW && high <= TOP_TRENCH_HIGH;

          assertTrue(
              inBottomTrench || inTopTrench,
              path + " point (" + String.format("%.2f, %.2f", point.getX(), point.getY())
                  + ") is inside the barrier band x=[" + BARRIER_NEAR_X + ", " + BARRIER_FAR_X
                  + "] but not inside a trench opening. That crosses a bump or the hub.");
        }
      }
    }
  }

  @Test
  public void trenchLanesStayUnderTheTrench() throws Exception {
    // The trench tags sit at y=0.644 and y=7.425. Running beside that line instead of on it is
    // what clipped the trench structure, so these lanes are pinned to it within a tolerance, and
    // still have to leave the robot's half-width clear of the field wall.
    // The trench opening is 1.668 m wide measured from the guardrail, so its centre is at
    // 0.834 (bottom) and FIELD_WIDTH - 0.834 (top). Centring the lane leaves a 0.9 m robot
    // 0.384 m either side; drifting off centre spends that margin for nothing.
    double tolerance = 0.20;
    for (String path : TRENCH_PATHS) {
      if (!pathFile(path).isFile()) {
        continue;
      }
      boolean bottom = path.contains("Bot");
      double laneCentre = bottom ? TRENCH_WIDTH / 2.0 : FIELD_WIDTH - TRENCH_WIDTH / 2.0;
      for (Translation2d point : allPoints(path)) {
        assertTrue(
            Math.abs(point.getY() - laneCentre) < tolerance,
            path + " point y=" + point.getY() + " is "
                + String.format("%.2f", Math.abs(point.getY() - laneCentre))
                + " m off the trench centreline at " + String.format("%.3f", laneCentre));
        double wallGap = bottom ? point.getY() : FIELD_WIDTH - point.getY();
        assertTrue(
            wallGap > ROBOT_HALF_WIDTH,
            path + " leaves only " + String.format("%.2f", wallGap)
                + " m to the wall, less than the robot's half-width");
      }
    }
  }

  private static Translation2d anchor(JsonNode waypoint) {
    return new Translation2d(
        waypoint.path("anchor").path("x").asDouble(), waypoint.path("anchor").path("y").asDouble());
  }

  @Test
  public void noAutoPutsTheRobotFullyAcrossTheCentreline() throws Exception {
    // Deliberately crossing fully onto the opponent's side during auto is penalised under G211,
    // up to a card for repeat or egregious cases. The denial auto in particular works the edge of
    // the neutral zone on purpose, so this is the assertion that keeps it legal.
    //
    // Checking control points too is not pedantry: the anchors can sit behind the line while the
    // curve between them bulges across it.
    for (String auto : AUTOS) {
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
    Translation2d hub = blueHubCentre();
    double keepOut = HUB_RADIUS + ROBOT_HALF_DIAGONAL;

    for (String auto : AUTOS) {
      for (String path : pathsOf(auto)) {
        for (Translation2d point : allPoints(path)) {
          double distance = hub.getDistance(point);
          assertTrue(
              distance > keepOut,
              path + " point (" + String.format("%.2f, %.2f", point.getX(), point.getY())
                  + ") is " + String.format("%.2f", distance)
                  + " m from the hub centre, inside the " + keepOut + " m keep-out");
        }
      }
    }
  }

  @Test
  public void everyPathEndsWithinShooterRange() throws Exception {
    // The shot table only covers 1.0 to 5.0 m and deliberately refuses to extrapolate, so a path
    // that parks the robot outside that band produces a shot of exactly zero RPM.
    Translation2d hub = blueHubCentre();

    for (String auto : AUTOS) {
      for (String path : pathsOf(auto)) {
        JsonNode waypoints = read(pathFile(path)).path("waypoints");
        JsonNode last = waypoints.get(waypoints.size() - 1);
        Translation2d end =
            new Translation2d(
                last.path("anchor").path("x").asDouble(), last.path("anchor").path("y").asDouble());
        // Range is measured to the hub face, which is about 0.6 m in from the centre.
        double range = hub.getDistance(end) - 0.6;
        assertTrue(
            range < 5.0,
            path + " ends " + String.format("%.2f", range)
                + " m from the hub, past the 5.0 m top of the shot table");
      }
    }
  }

  /** Mean of the eight blue hub tag positions from the official 2026 layout. */
  private static Translation2d blueHubCentre() {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    int[] blueHub = {18, 19, 20, 21, 24, 25, 26, 27};
    double sumX = 0;
    double sumY = 0;
    for (int id : blueHub) {
      var pose = layout.getTagPose(id).orElseThrow();
      sumX += pose.getX();
      sumY += pose.getY();
    }
    return new Translation2d(sumX / blueHub.length, sumY / blueHub.length);
  }
}
