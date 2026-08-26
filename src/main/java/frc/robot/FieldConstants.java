package frc.robot;

import java.util.HashMap;
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
 * 2026 REBUILT field facts, taken from the official WPILib AprilTag layout rather than typed in by
 * hand.
 *
 * <h2>Tag map</h2>
 *
 * <p>The field carries 32 tags of the 36h11 family, and they are NOT interchangeable -- they sit at
 * three different heights, which is what makes getting the ID sets right a ranging problem and not
 * just a filtering nicety:
 *
 * <table>
 *   <caption>2026 REBUILT AprilTag groups</caption>
 *   <tr><th>Structure</th><th>Blue IDs</th><th>Red IDs</th><th>Height</th></tr>
 *   <tr><td>HUB</td><td>18-21, 24-27</td><td>2-5, 8-11</td><td>44.25 in</td></tr>
 *   <tr><td>TRENCH</td><td>17, 22, 23, 28</td><td>1, 6, 7, 12</td><td>35 in</td></tr>
 *   <tr><td>TOWER</td><td>31, 32</td><td>15, 16</td><td>21.75 in</td></tr>
 *   <tr><td>OUTPOST</td><td>29, 30</td><td>13, 14</td><td>21.75 in</td></tr>
 * </table>
 *
 * <p><b>Tag 1 is a TRENCH tag, not the HUB.</b> Aiming the shooter at it means ranging off a target
 * mounted 9.25 in lower than the hub, in a completely different place on the field.
 */
public final class FieldConstants {

  private FieldConstants() {
  }

  /**
   * Official 2026 tag layout.
   *
   * <p>Switch to {@link AprilTagFields#k2026RebuiltAndymark} if the event runs AndyMark field
   * hardware; the two differ by small tag placement tolerances.
   */
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();

  // ---- Tag groups ----------------------------------------------------------

  public static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
  public static final Set<Integer> RED_HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
  public static final Set<Integer> ALL_HUB_TAG_IDS =
      Set.of(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

  public static final Set<Integer> BLUE_TRENCH_TAG_IDS = Set.of(17, 22, 23, 28);
  public static final Set<Integer> RED_TRENCH_TAG_IDS = Set.of(1, 6, 7, 12);
  public static final Set<Integer> ALL_TRENCH_TAG_IDS = Set.of(1, 6, 7, 12, 17, 22, 23, 28);

  public static final Set<Integer> BLUE_TOWER_TAG_IDS = Set.of(31, 32);
  public static final Set<Integer> RED_TOWER_TAG_IDS = Set.of(15, 16);
  public static final Set<Integer> ALL_TOWER_TAG_IDS = Set.of(15, 16, 31, 32);

  public static final Set<Integer> BLUE_OUTPOST_TAG_IDS = Set.of(29, 30);
  public static final Set<Integer> RED_OUTPOST_TAG_IDS = Set.of(13, 14);
  public static final Set<Integer> ALL_OUTPOST_TAG_IDS = Set.of(13, 14, 29, 30);

  // ---- Derived geometry ----------------------------------------------------

  /** Tag centre height above the carpet, keyed by ID, read straight out of the official layout. */
  private static final Map<Integer, Double> TAG_HEIGHTS_METERS = buildTagHeightMap();

  /** Fallback height used if an unknown tag ID is somehow reported: the HUB height. */
  public static final double DEFAULT_TAG_HEIGHT_METERS = tagHeightOrDefault(20, 1.124);

  public static final Translation2d BLUE_HUB_CENTER = centroidOf(BLUE_HUB_TAG_IDS);
  public static final Translation2d RED_HUB_CENTER = centroidOf(RED_HUB_TAG_IDS);

  private static Map<Integer, Double> buildTagHeightMap() {
    Map<Integer, Double> heights = new HashMap<>();
    for (AprilTag tag : FIELD_LAYOUT.getTags()) {
      heights.put(tag.ID, tag.pose.getZ());
    }
    return Map.copyOf(heights);
  }

  private static double tagHeightOrDefault(int tagId, double fallbackMeters) {
    Double height = TAG_HEIGHTS_METERS.get(tagId);
    return height == null ? fallbackMeters : height;
  }

  /** Average field position of a set of tags. Used to locate a structure from its tags. */
  private static Translation2d centroidOf(Set<Integer> tagIds) {
    double sumX = 0.0;
    double sumY = 0.0;
    int count = 0;
    for (int tagId : tagIds) {
      Optional<Pose3d> pose = FIELD_LAYOUT.getTagPose(tagId);
      if (pose.isPresent()) {
        sumX += pose.get().getX();
        sumY += pose.get().getY();
        count++;
      }
    }
    return count == 0 ? Translation2d.kZero : new Translation2d(sumX / count, sumY / count);
  }

  /**
   * Height above the carpet of a given tag's centre, in meters.
   *
   * <p>Range is computed by walking the camera ray out to the tag's height plane, so using the
   * wrong height here scales every distance measurement. A HUB tag read as a TRENCH tag, or vice
   * versa, is a 9.25 in height error and skews range by roughly 25 percent.
   */
  public static double tagHeightMeters(int tagId) {
    return tagHeightOrDefault(tagId, DEFAULT_TAG_HEIGHT_METERS);
  }

  /** Current alliance, defaulting to blue before the driver station reports one. */
  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * HUB tags to aim at.
   *
   * <p>Before the driver station reports an alliance this returns both hubs, so the turret still
   * finds something to track in the pit. Once an alliance is known it narrows to that alliance's
   * hub only -- tracking the opponent's hub during a match would be worse than not tracking at all.
   */
  public static Set<Integer> hubTagIds() {
    Optional<Alliance> reported = DriverStation.getAlliance();
    if (reported.isEmpty()) {
      return ALL_HUB_TAG_IDS;
    }
    return reported.get() == Alliance.Red ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS;
  }

  /** Field position of the hub this alliance scores in. */
  public static Translation2d hubCenter() {
    return isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
  }

  /** TOWER tags for this alliance, used when lining up to climb. */
  public static Set<Integer> towerTagIds() {
    Optional<Alliance> reported = DriverStation.getAlliance();
    if (reported.isEmpty()) {
      return ALL_TOWER_TAG_IDS;
    }
    return reported.get() == Alliance.Red ? RED_TOWER_TAG_IDS : BLUE_TOWER_TAG_IDS;
  }

  /** OUTPOST tags for this alliance. */
  public static Set<Integer> outpostTagIds() {
    Optional<Alliance> reported = DriverStation.getAlliance();
    if (reported.isEmpty()) {
      return ALL_OUTPOST_TAG_IDS;
    }
    return reported.get() == Alliance.Red ? RED_OUTPOST_TAG_IDS : BLUE_OUTPOST_TAG_IDS;
  }
}
