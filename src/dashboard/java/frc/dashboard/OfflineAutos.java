package frc.dashboard;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.node.ObjectNode;

import frc.robot.auto.AutoPreviewBuilder;

/**
 * The autonomous routines in this project's own files, for looking at with no robot connected.
 *
 * <p>Built by the same {@link AutoPreviewBuilder} the robot uses, from the PathPlanner files in
 * {@code src/main/deploy}, so the picture is the one the robot would publish from the same files.
 * Once a robot connects, its own catalog takes over: that is what is actually deployed.
 *
 * <p>Only PathPlanner routines are here. The robot also offers a couple of routines defined in
 * code (a timed shot, do nothing), which appear once it is connected.
 */
final class OfflineAutos {

  /**
   * Named-command timings, the same values RobotContainer registers with the robot's catalog. The
   * dashboard cannot read robot code, so these are kept in step by hand.
   */
  private static final double SHOOT_SECONDS = 3.5;
  private static final double ALIGN_SECONDS = 2.0;
  private static final double FEED_CORRAL_SECONDS = 2.0;

  /** Chooser labels drop the generated routines' "R26 " prefix; so do these, so they match. */
  private static final String GENERATED_PREFIX = "R26 ";

  /** @param catalogJson the catalog, or empty when there is none; @param problem why, or null */
  record Result(String catalogJson, String problem, int count) {
  }

  private OfflineAutos() {
  }

  /** Reads and previews every routine. Takes a moment; call it off the Swing thread. */
  static Result load() {
    try {
      if (!AutoPreviewBuilder.pathPlannerDirectory().isDirectory()) {
        return new Result("", "No autos on this laptop. Start the dashboard from the robot "
            + "project folder (Dashboard.bat does) to look at them without a robot.", 0);
      }
      List<String> files = AutoPreviewBuilder.autoFiles();
      if (files.isEmpty()) {
        return new Result("", "The project has no autos in src/main/deploy/pathplanner/autos.", 0);
      }

      AutoPreviewBuilder builder = new AutoPreviewBuilder(System.err::println);
      builder.setStepSeconds("Shoot", SHOOT_SECONDS);
      builder.setStepSeconds("AlignToHub", ALIGN_SECONDS);
      builder.setStepSeconds("FeedCorral", FEED_CORRAL_SECONDS);

      List<ObjectNode> routines = new ArrayList<>();
      for (String file : files) {
        routines.add(builder.pathPlanner(label(file), file, false));
      }
      return new Result(builder.catalog(routines), null, routines.size());
    } catch (Throwable t) {
      // A missing native library or a malformed file must not take the dashboard down.
      return new Result("", "Couldn't read the autos on this laptop: " + t, 0);
    }
  }

  static String label(String file) {
    return file.startsWith(GENERATED_PREFIX) ? file.substring(GENERATED_PREFIX.length()) : file;
  }
}
