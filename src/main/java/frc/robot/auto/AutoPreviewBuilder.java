package frc.robot.auto;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

/**
 * Builds the preview of an autonomous routine that the dashboard draws: its steps in order, with
 * every path as timed samples of the trajectory PathPlanner will follow.
 *
 * <p>Shared, file for file, between the robot and the dashboard. The robot ({@link AutoCatalog})
 * builds previews from the files deployed to it and publishes them, which is the source of truth
 * once a robot is connected. The dashboard compiles this same class (see the {@code dashboard}
 * source set in build.gradle) and builds previews from the project's own deploy folder, so the
 * routines can be looked at in the pits or at home with no robot at all. Same code, same files,
 * same picture.
 *
 * <p>That is why it may only depend on libraries, never on other robot code, and why problems are
 * reported through a callback rather than the Driver Station, which a laptop does not have.
 *
 * <p>Files are read from WPILib's deploy directory: {@code /home/lvuser/deploy} on the robot, and
 * {@code src/main/deploy} under the working directory anywhere else, which is the project folder
 * when the dashboard is started with Dashboard.bat or {@code ./gradlew dashboard}.
 */
public final class AutoPreviewBuilder {

  /** Spacing of the points sent per path. Smooth enough to draw, small enough not to matter. */
  private static final double SAMPLE_SECONDS = 0.05;

  private final Map<String, Double> stepSeconds = new HashMap<>();
  private final Consumer<String> warn;
  private final ObjectMapper json = new ObjectMapper();

  private RobotConfig robotConfig;
  private boolean robotConfigTried = false;

  /** @param warn where to report a routine whose preview could not be built */
  public AutoPreviewBuilder(Consumer<String> warn) {
    this.warn = warn;
  }

  /** The folder routines are read from, under the deploy directory. */
  public static File pathPlannerDirectory() {
    return new File(Filesystem.getDeployDirectory(), "pathplanner");
  }

  /** Every {@code .auto} file name (without the extension) in the deploy folder, sorted. */
  public static List<String> autoFiles() {
    File[] files = new File(pathPlannerDirectory(), "autos")
        .listFiles((dir, name) -> name.endsWith(".auto"));
    List<String> names = new ArrayList<>();
    if (files != null) {
      for (File file : files) {
        names.add(file.getName().substring(0, file.getName().length() - ".auto".length()));
      }
    }
    names.sort(String::compareTo);
    return names;
  }

  /**
   * How long a named command is expected to take, for the preview's timeline.
   *
   * <p>Only the ones that hold the routine up need this. Anything unset counts as instant, which
   * is right for the ones that only start a mechanism or run alongside a path.
   */
  public void setStepSeconds(String namedCommand, double seconds) {
    stepSeconds.put(namedCommand, seconds);
  }

  /**
   * Preview of a PathPlanner routine. Never throws: a routine whose preview cannot be built
   * carries the error instead, so the drive team sees why there is no picture.
   */
  public ObjectNode pathPlanner(String label, String autoFile, boolean isDefault) {
    ObjectNode out = json.createObjectNode();
    out.put("label", label);
    out.put("default", isDefault);
    out.put("file", autoFile);
    try {
      out.set("steps", stepsOf(autoFile));
    } catch (Exception e) {
      String message = e.getClass().getSimpleName()
          + (e.getMessage() == null ? "" : ": " + e.getMessage());
      out.put("error", message);
      out.putArray("steps");
      warn.accept("[AUTO] No preview for '" + label + "': " + message);
    }
    return out;
  }

  /**
   * Preview of a routine that never drives.
   *
   * @param title what it does, or null for a routine that does nothing at all
   */
  public ObjectNode inPlace(String label, boolean isDefault, String title, double seconds) {
    ObjectNode out = json.createObjectNode();
    out.put("label", label);
    out.put("default", isDefault);
    ArrayNode steps = out.putArray("steps");
    if (title != null) {
      ObjectNode step = steps.addObject();
      step.put("kind", "action");
      step.put("title", title);
      step.put("seconds", seconds);
      out.put("note", "Runs from wherever the robot is placed. Does not drive.");
    } else {
      out.put("note", "The robot stays still for the whole autonomous period.");
    }
    return out;
  }

  /** The catalog JSON the dashboard reads: {@code {"version":1,"routines":[...]}}. */
  public String catalog(List<ObjectNode> routines) {
    ObjectNode catalog = json.createObjectNode();
    catalog.put("version", 1);
    ArrayNode array = catalog.putArray("routines");
    routines.forEach(array::add);
    return catalog.toString();
  }

  /** Flattens one {@code .auto} file into the ordered steps a person would describe it as. */
  public ArrayNode stepsOf(String autoFile) throws Exception {
    File file = new File(pathPlannerDirectory(), "autos/" + autoFile + ".auto");
    JsonNode auto = json.readTree(file);
    ArrayNode steps = json.createArrayNode();
    flatten(auto.path("command"), steps);
    return steps;
  }

  private void flatten(JsonNode command, ArrayNode steps) throws Exception {
    String type = command.path("type").asText("");
    JsonNode data = command.path("data");
    switch (type) {
      case "sequential":
        for (JsonNode child : data.path("commands")) {
          flatten(child, steps);
        }
        break;
      case "parallel":
      case "deadline":
      case "race":
        flattenGroup(type, data.path("commands"), steps);
        break;
      case "path":
        steps.add(driveStep(data.path("pathName").asText(""), List.of()));
        break;
      case "named":
        steps.add(actionStep(List.of(data.path("name").asText("")), "sequential"));
        break;
      case "wait": {
        ObjectNode step = steps.addObject();
        step.put("kind", "wait");
        step.put("seconds", data.path("waitTime").asDouble(0.0));
        break;
      }
      default:
        // Unknown command types are skipped rather than failing the whole preview.
        break;
    }
  }

  /**
   * Commands that run together, shown as one step: the path being driven, and what runs with it.
   *
   * <p>That is how the drive team describes it, too. "Drive under the trench with the intake
   * down" is one thing, not a path and a separate intake command that happen to overlap.
   */
  private void flattenGroup(String type, JsonNode children, ArrayNode steps) throws Exception {
    List<String> paths = new ArrayList<>();
    List<String> alongside = new ArrayList<>();
    collectNames(children, paths, alongside);

    if (paths.isEmpty()) {
      steps.add(actionStep(alongside, type));
      return;
    }
    steps.add(driveStep(paths.get(0), alongside));
    // Two paths in one parallel group cannot both drive. Shown in order so neither goes missing.
    for (int i = 1; i < paths.size(); i++) {
      steps.add(driveStep(paths.get(i), List.of()));
    }
  }

  private static void collectNames(JsonNode commands, List<String> paths, List<String> named) {
    for (JsonNode child : commands) {
      String type = child.path("type").asText("");
      JsonNode data = child.path("data");
      if ("path".equals(type)) {
        paths.add(data.path("pathName").asText(""));
      } else if ("named".equals(type)) {
        named.add(data.path("name").asText(""));
      } else if (data.has("commands")) {
        collectNames(data.path("commands"), paths, named);
      }
    }
  }

  private ObjectNode actionStep(List<String> names, String groupType) {
    ObjectNode step = json.createObjectNode();
    step.put("kind", "action");
    ArrayNode nameArray = step.putArray("names");
    names.forEach(nameArray::add);

    // How long the group holds the routine up depends on how it ends.
    double seconds = 0.0;
    if (!names.isEmpty()) {
      if ("deadline".equals(groupType) || "sequential".equals(groupType)) {
        seconds = secondsOf(names.get(0));
      } else if ("race".equals(groupType)) {
        seconds = names.stream().mapToDouble(this::secondsOf).min().orElse(0.0);
      } else {
        seconds = names.stream().mapToDouble(this::secondsOf).max().orElse(0.0);
      }
    }
    step.put("seconds", seconds);
    return step;
  }

  private double secondsOf(String namedCommand) {
    return stepSeconds.getOrDefault(namedCommand, 0.0);
  }

  /**
   * One driven path, as timed samples of {@code [t, x, y, headingDegrees]}.
   *
   * <p>Sampled from the trajectory PathPlanner will actually follow, so the timing and the
   * robot's rotation along the way are the real ones, not a guess from the path's length.
   */
  private ObjectNode driveStep(String pathName, List<String> alongside) throws Exception {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    ObjectNode step = json.createObjectNode();
    step.put("kind", "drive");
    step.put("name", pathName);
    ArrayNode with = step.putArray("with");
    alongside.forEach(with::add);
    ArrayNode points = step.putArray("points");

    Optional<PathPlannerTrajectory> trajectory =
        config() == null ? Optional.empty() : path.getIdealTrajectory(config());

    if (trajectory.isPresent()) {
      double total = trajectory.get().getTotalTimeSeconds();
      for (double t = 0.0; ; t += SAMPLE_SECONDS) {
        double at = Math.min(t, total);
        PathPlannerTrajectoryState state = trajectory.get().sample(at);
        addPoint(points, at, state.pose);
        if (at >= total) {
          break;
        }
      }
      step.put("seconds", total);
      return step;
    }

    // No ideal starting state means no trajectory. The shape is still worth drawing, timed at
    // the path's own speed limit so the timeline is at least in the right ballpark.
    List<Pose2d> poses = path.getPathPoses();
    double speed = Math.max(0.5, path.getGlobalConstraints().maxVelocityMPS());
    double t = 0.0;
    for (int i = 0; i < poses.size(); i++) {
      if (i > 0) {
        t += poses.get(i).getTranslation().getDistance(poses.get(i - 1).getTranslation()) / speed;
      }
      addPoint(points, t, poses.get(i));
    }
    step.put("seconds", t);
    step.put("estimated", true);
    return step;
  }

  private static void addPoint(ArrayNode points, double t, Pose2d pose) {
    points.add(round(t));
    points.add(round(pose.getX()));
    points.add(round(pose.getY()));
    points.add(round(pose.getRotation().getDegrees()));
  }

  private static double round(double value) {
    return Math.round(value * 1000.0) / 1000.0;
  }

  private RobotConfig config() {
    if (!robotConfigTried) {
      robotConfigTried = true;
      try {
        robotConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        warn.accept("[AUTO] PathPlanner robot config unavailable; auto previews use estimated "
            + "timing.");
      }
    }
    return robotConfig;
  }
}
