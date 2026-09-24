package frc.robot.vision;

import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The 2026 REBUILT AprilTag map.
 *
 * <p>32 tags, family 36h11, IDs 1-32. They are not interchangeable: each belongs to a structure
 * with a specific job and a specific mounting height.
 *
 * <pre>
 *        RED half                          BLUE half
 *   +----------------------+          +----------------------+
 *   | TRENCH   1  6  7 12  |          | TRENCH  17 22 23 28  |
 *   | HUB      2  3  4  5  |          | HUB     18 19 20 21  |
 *   |          8  9 10 11  |          |         24 25 26 27  |
 *   | TOWER   15 16        |          | TOWER   31 32        |
 *   | OUTPOST 13 14        |          | OUTPOST 29 30        |
 *   +----------------------+          +----------------------+
 * </pre>
 *
 * <p>Membership below is asserted by hand because that is the team-facing truth, but every entry is
 * cross-checked at class load against the official WPILib layout: if a tag's real height does not
 * match the function claimed for it, construction fails loudly rather than shipping a silent
 * ranging error to a match.
 */
public final class FieldTags {

  private FieldTags() {
  }

  /**
   * Official 2026 layout.
   *
   * <p>Switch to {@link AprilTagFields#k2026RebuiltAndymark} if the event runs AndyMark field
   * hardware; the two differ by small tag placement tolerances.
   */
  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double FIELD_LENGTH_METERS = LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = LAYOUT.getFieldWidth();

  // ---- Tag groups, straight off the field drawing ---------------------------

  public static final Set<Integer> RED_HUB = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
  public static final Set<Integer> BLUE_HUB = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

  public static final Set<Integer> RED_TRENCH = Set.of(1, 6, 7, 12);
  public static final Set<Integer> BLUE_TRENCH = Set.of(17, 22, 23, 28);

  public static final Set<Integer> RED_TOWER = Set.of(15, 16);
  public static final Set<Integer> BLUE_TOWER = Set.of(31, 32);

  public static final Set<Integer> RED_OUTPOST = Set.of(13, 14);
  public static final Set<Integer> BLUE_OUTPOST = Set.of(29, 30);

  public static final Set<Integer> ALL_HUB = union(RED_HUB, BLUE_HUB);
  public static final Set<Integer> ALL_TRENCH = union(RED_TRENCH, BLUE_TRENCH);
  public static final Set<Integer> ALL_TOWER = union(RED_TOWER, BLUE_TOWER);
  public static final Set<Integer> ALL_OUTPOST = union(RED_OUTPOST, BLUE_OUTPOST);

  private static final Map<Integer, TagFunction> FUNCTION_BY_ID = buildFunctionMap();
  private static final Map<Integer, Alliance> ALLIANCE_BY_ID = buildAllianceMap();

  /** Centre of each alliance's HUB in field coordinates, averaged from its eight tags. */
  public static final Translation2d RED_HUB_CENTER = centroidOf(RED_HUB);
  public static final Translation2d BLUE_HUB_CENTER = centroidOf(BLUE_HUB);

  static {
    verifyAgainstOfficialLayout();
  }

  // ---- Lookups --------------------------------------------------------------

  /** What structure a tag belongs to, or {@link TagFunction#UNKNOWN} for an off-field ID. */
  public static TagFunction functionOf(int tagId) {
    return FUNCTION_BY_ID.getOrDefault(tagId, TagFunction.UNKNOWN);
  }

  /** Which alliance owns a tag, or empty for an off-field ID. */
  public static Optional<Alliance> allianceOf(int tagId) {
    return Optional.ofNullable(ALLIANCE_BY_ID.get(tagId));
  }

  /** True when the ID is one of the 32 real field tags. */
  public static boolean isFieldTag(int tagId) {
    return FUNCTION_BY_ID.containsKey(tagId);
  }

  /** Height of a tag's centre above the carpet, in meters. */
  public static double heightMeters(int tagId) {
    return functionOf(tagId).heightMeters();
  }

  /** Known field pose of a tag, straight from the official layout. */
  public static Optional<Pose3d> poseOf(int tagId) {
    return LAYOUT.getTagPose(tagId);
  }

  /** Human-readable label for logs and dashboards, e.g. {@code "20 (BLUE HUB)"}. */
  public static String describe(int tagId) {
    if (!isFieldTag(tagId)) {
      return tagId + " (NOT A 2026 FIELD TAG)";
    }
    return String.format(
        "%d (%s %s)",
        tagId,
        // Alliance.name() is "Blue"/"Red"; uppercased so it reads consistently beside the
        // all-caps TagFunction in log lines.
        allianceOf(tagId).map(a -> a.name().toUpperCase()).orElse("?"),
        functionOf(tagId));
  }

  // ---- Alliance-aware accessors ---------------------------------------------

  /**
   * Current alliance as reported by the driver station.
   *
   * <p>Empty before the station connects, which matters: filtering to one alliance's tags before
   * knowing the alliance would silently reject everything.
   */
  public static Optional<Alliance> alliance() {
    return DriverStation.getAlliance();
  }

  public static boolean isRed() {
    return alliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * HUB tags to shoot at.
   *
   * <p>Returns BOTH hubs until the driver station reports an alliance, so the robot still sees
   * something on a bench or a practice field. Once an alliance is known it narrows to that hub —
   * aiming at the opponent's goal during a match is worse than not aiming at all.
   */
  public static Set<Integer> hubTags() {
    return alliance().map(a -> a == Alliance.Red ? RED_HUB : BLUE_HUB).orElse(ALL_HUB);
  }

  /** TOWER tags for climbing, narrowed to this alliance once one is reported. */
  public static Set<Integer> towerTags() {
    return alliance().map(a -> a == Alliance.Red ? RED_TOWER : BLUE_TOWER).orElse(ALL_TOWER);
  }

  /** OUTPOST tags for loading, narrowed to this alliance once one is reported. */
  public static Set<Integer> outpostTags() {
    return alliance().map(a -> a == Alliance.Red ? RED_OUTPOST : BLUE_OUTPOST).orElse(ALL_OUTPOST);
  }

  /** Field position of the hub this alliance scores in. */
  public static Translation2d hubCenter() {
    return isRed() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
  }

  // ---- Construction ---------------------------------------------------------

  private static Set<Integer> union(Set<Integer> a, Set<Integer> b) {
    Set<Integer> out = new HashSet<>(a);
    out.addAll(b);
    return Collections.unmodifiableSet(out);
  }

  private static Map<Integer, TagFunction> buildFunctionMap() {
    Map<Integer, TagFunction> map = new HashMap<>();
    Map<TagFunction, Set<Integer>> groups = new EnumMap<>(TagFunction.class);
    groups.put(TagFunction.HUB, ALL_HUB);
    groups.put(TagFunction.TRENCH, ALL_TRENCH);
    groups.put(TagFunction.TOWER, ALL_TOWER);
    groups.put(TagFunction.OUTPOST, ALL_OUTPOST);

    groups.forEach((function, ids) -> ids.forEach(id -> map.put(id, function)));
    return Map.copyOf(map);
  }

  private static Map<Integer, Alliance> buildAllianceMap() {
    Map<Integer, Alliance> map = new HashMap<>();
    for (int id : union(union(RED_HUB, RED_TRENCH), union(RED_TOWER, RED_OUTPOST))) {
      map.put(id, Alliance.Red);
    }
    for (int id : union(union(BLUE_HUB, BLUE_TRENCH), union(BLUE_TOWER, BLUE_OUTPOST))) {
      map.put(id, Alliance.Blue);
    }
    return Map.copyOf(map);
  }

  private static Translation2d centroidOf(Set<Integer> tagIds) {
    double sumX = 0.0;
    double sumY = 0.0;
    int count = 0;
    for (int id : tagIds) {
      Optional<Pose3d> pose = LAYOUT.getTagPose(id);
      if (pose.isPresent()) {
        sumX += pose.get().getX();
        sumY += pose.get().getY();
        count++;
      }
    }
    return count == 0 ? Translation2d.kZero : new Translation2d(sumX / count, sumY / count);
  }

  /**
   * Cross-checks every hand-entered group against the real layout.
   *
   * <p>Catches the failure mode that is otherwise invisible until a match: a tag listed under the
   * wrong structure still detects fine and still produces a number, it is just the wrong distance.
   * Comparing the claimed height against the official pose turns that into a startup error.
   */
  private static void verifyAgainstOfficialLayout() {
    StringBuilder problems = new StringBuilder();

    for (AprilTag tag : LAYOUT.getTags()) {
      if (!FUNCTION_BY_ID.containsKey(tag.ID)) {
        problems.append(String.format("%n  tag %d exists on the field but is not grouped", tag.ID));
      }
    }

    FUNCTION_BY_ID.forEach((id, function) -> {
      Optional<Pose3d> pose = LAYOUT.getTagPose(id);
      if (pose.isEmpty()) {
        problems.append(String.format("%n  tag %d is grouped but not on the 2026 field", id));
        return;
      }
      double actualMeters = pose.get().getZ();
      double claimedMeters = function.heightMeters();
      if (Math.abs(actualMeters - claimedMeters) > 0.02) {
        problems.append(String.format(
            "%n  tag %d is grouped as %s (%.3f m) but the field puts it at %.3f m",
            id, function, claimedMeters, actualMeters));
      }
    });

    if (problems.length() > 0) {
      throw new IllegalStateException(
          "FieldTags does not match the official 2026 layout:" + problems);
    }
  }
}
