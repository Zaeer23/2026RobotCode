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
    "Centre 2 Ball",
    "Bottom Trench 3 Ball",
    "Top Trench 3 Ball",
    "Shoot and Leave",
    "Neutral Zone Denial"
  };

  private static final double FIELD_LENGTH = 16.541;
  private static final double FIELD_WIDTH = 8.069;
  private static final double CENTRELINE_X = FIELD_LENGTH / 2.0;

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
        for (Translation2d point : allPoints(path)) {
          assertTrue(
              point.getX() > ROBOT_HALF_DIAGONAL
                  && point.getX() < FIELD_LENGTH - ROBOT_HALF_DIAGONAL,
              path + " point x=" + point.getX() + " puts the robot through a wall");
          assertTrue(
              point.getY() > ROBOT_HALF_DIAGONAL
                  && point.getY() < FIELD_WIDTH - ROBOT_HALF_DIAGONAL,
              path + " point y=" + point.getY() + " puts the robot through a wall");
        }
      }
    }
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
