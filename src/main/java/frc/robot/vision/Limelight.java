package frc.robot.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;


import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.util.BoundedTcpForwarder;

/**
 * Limelight AprilTag reader.
 *
 * <p>Written to be boring. Every piece of cleverness in the previous versions turned into a
 * failure that looked like a dead camera or a dead turret, so this one does the minimum and says
 * loudly what it is doing.
 *
 * <h2>Rules, each one paid for</h2>
 *
 * <ol>
 *   <li><b>Nothing invalid leaves this class.</b> A {@link VisionTarget} is only constructed from
 *       finite, in-range numbers. A NaN that escapes passes silently through every downstream
 *       clamp — comparisons against NaN are all false — and ends up in {@code spark.set()}.
 *   <li><b>Targets are debounced.</b> A lock needs several consecutive good frames and survives a
 *       few bad ones. One spurious detection must never fling the turret.
 *   <li><b>Filtering is the caller's choice, and visible when it bites.</b> Aiming used to be
 *       hard-wired to this alliance's hub tags, so any other tag was dropped in silence: camera
 *       sees it, dashboard shows it, turret ignores it. That is reported now.
 *   <li><b>The camera's own configuration is left alone except the pipeline.</b> No tag-ID filter
 *       is ever pushed to the hardware; that persists across reboots and has twice made a working
 *       camera look blind.
 * </ol>
 */
public class Limelight extends SubsystemBase {

  /** Which tags the turret is allowed to aim at. */
  public enum TargetFilter {
    /** Any valid field tag. The right setting for bring-up and bench testing. */
    ANY_TAG,
    /** Only HUB tags belonging to this alliance. The right setting for a match. */
    ALLIANCE_HUB
  }

  // rawfiducials layout: 7 doubles per tag.
  private static final int STRIDE = 7;
  private static final int I_ID = 0;
  private static final int I_TX = 1;
  private static final int I_TY = 2;
  private static final int I_TA = 3;
  private static final int I_DIST_CAMERA = 4;

  /** Ranges outside this are physically impossible on a 16 m field and are rejected. */
  private static final double MIN_RANGE_METERS = 0.15;
  private static final double MAX_RANGE_METERS = 20.0;

  /** tx/ty beyond the widest Limelight field of view are nonsense. */
  private static final double MAX_ANGLE_DEGREES = 60.0;

  /** Consecutive good frames before a target is handed out. */
  private static final int FRAMES_TO_ACQUIRE = 3;

  /** Consecutive missing frames before a lock is dropped. */
  private static final int FRAMES_TO_DROP = 5;

  /** No publish for this long means the camera is gone. */
  private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;

  private static final double WARN_PERIOD_SECONDS = 5.0;

  private final String name;
  private final NetworkTable table;
  private final NetworkTableEntry tvEntry;
  private final NetworkTableEntry tlEntry;
  private final NetworkTableEntry clEntry;
  private final NetworkTableEntry hbEntry;
  private final NetworkTableEntry getPipeEntry;
  private final NetworkTableEntry pipelineEntry;
  private final NetworkTableEntry rawFiducialsEntry;

  private TargetFilter filter = TargetFilter.ANY_TAG;

  /**
   * Every tag id this camera has reported since boot, and every id it reported that is not on
   * the 2026 field.
   *
   * <p>Kept because the two most likely ways a camera is "wrong for 2026" both look identical
   * from the robot: no targets. Either the pipeline is filtering ids to a previous season, in
   * which case the high tags simply never arrive, or it is on the wrong tag family, in which
   * case nothing arrives. Listing what DID arrive separates them.
   */
  private final java.util.Set<Integer> seenTagIds = new java.util.TreeSet<>();
  private final java.util.Set<Integer> rejectedTagIds = new java.util.TreeSet<>();

  private List<VisionTarget> visibleTargets = List.of();
  private Optional<VisionTarget> lockedTarget = Optional.empty();
  private int goodFrames = 0;
  private int missedFrames = 0;

  private boolean connected = false;
  private double lastPublishSeconds = -1.0;
  private long lastHeartbeat = Long.MIN_VALUE;
  private long lastTvChange = Long.MIN_VALUE;

  private double lastWarnSeconds = -1.0;
  private boolean announcedConnection = false;
  private String status = "STARTING";

  public Limelight(String name) {
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.tvEntry = table.getEntry("tv");
    this.tlEntry = table.getEntry("tl");
    this.clEntry = table.getEntry("cl");
    this.hbEntry = table.getEntry("hb");
    this.getPipeEntry = table.getEntry("getpipe");
    this.pipelineEntry = table.getEntry("pipeline");
    this.rawFiducialsEntry = table.getEntry("rawfiducials");
    setName("Limelight[" + name + "]");
  }

  // ---- API ------------------------------------------------------------------

  /** True when the camera is publishing to NetworkTables right now. */
  public boolean isConnected() {
    return connected;
  }

  /** Every valid tag currently visible, nearest first. Unfiltered. */
  public List<VisionTarget> getVisibleTargets() {
    return visibleTargets;
  }

  /**
   * The target to aim at: present only when a debounced lock is held.
   *
   * <p>Empty means the turret must not move. There is no "coast to the last known angle" here —
   * driving a turret off a target that is no longer there is how it ends up moving on its own.
   */
  public Optional<VisionTarget> getTarget() {
    return lockedTarget;
  }

  /** True when a debounced target is available. */
  public boolean hasTarget() {
    return lockedTarget.isPresent();
  }

  /**
   * The nearest visible HUB tag, ignoring every other tag on the field.
   *
   * <p>This exists because {@link #getTarget()} will happily hand back a TRENCH tag. The trench is
   * a structure covered in AprilTags that we deliberately park right next to in the neutral zone,
   * so from a shooting position its tags are closer and squarer to the camera than the hub's. Aim
   * and range taken from {@link #getTarget()} there would point the turret at the trench and
   * compute the distance to it, and the robot would fire into the side of the tunnel.
   *
   * <p>Deliberately not restricted to our own alliance's hub. Filtering by alliance is what made
   * the turret ignore a tag that was being held up in front of it during testing, and either hub
   * is a real hub, at a real HUB mounting height, so the ranging is correct for both.
   */
  public Optional<VisionTarget> getHubTarget() {
    // Nearest wins when several hub tags are in view, which is the normal case: the hub carries
    // eight and a shooting position usually sees two or three of them.
    //
    // Stated as an explicit min rather than relying on visibleTargets already being sorted by
    // range. It is, but that is done a hundred lines away, and a findFirst() that silently means
    // "closest" only by coincidence of ordering is the kind of thing that breaks quietly when
    // someone changes the sort.
    return visibleTargets.stream()
        .filter(target -> FieldTags.functionOf(target.id).isScoringTarget())
        .min(java.util.Comparator.comparingDouble(target -> target.rangeMeters));
  }

  /**
   * Prints which tag ids this camera has actually produced, and which it produced that are not
   * on the 2026 field.
   *
   * <p>"The tags are wrong for this season" looks identical from the robot to "the camera sees
   * nothing", because an id that is not on the field is dropped before anything downstream hears
   * about it. This separates the cases:
   *
   * <ul>
   *   <li><b>Nothing at all</b> — the pipeline is not an AprilTag pipeline, or it is on the wrong
   *       family. 2026 is 36h11.</li>
   *   <li><b>Only low ids, never above 22</b> — the pipeline is filtering ids to last season's
   *       range. 2026 runs 1 to 32, and the blue hub needs 18 to 27.</li>
   *   <li><b>Ids listed as rejected</b> — the camera is reporting tags that do not exist on this
   *       field at all.</li>
   * </ul>
   */
  public void reportTagIdCoverage() {
    System.out.println("  tag ids seen since boot: "
        + (seenTagIds.isEmpty() ? "NONE" : seenTagIds.toString()));

    if (!rejectedTagIds.isEmpty()) {
      System.out.println("  REJECTED ids not on the 2026 field: " + rejectedTagIds);
      System.out.println("    The pipeline is reporting tags from another field map.");
    }
    if (seenTagIds.isEmpty()) {
      System.out.println("    No valid tag has ever arrived. Check the pipeline is AprilTag type");
      System.out.println("    and family 36h11, and that the id filter is not set.");
    } else {
      int highest = java.util.Collections.max(seenTagIds);
      if (highest <= 22) {
        System.out.println("    Nothing above id 22 has ever been seen. 2026 uses 1-32 and the");
        System.out.println("    BLUE hub is 18-27, so a filter stuck on last season's range");
        System.out.println("    would look exactly like this.");
      }
    }
  }

  /** True when an actual HUB tag is in view, as opposed to any tag at all. */
  public boolean hasHubTarget() {
    return getHubTarget().isPresent();
  }

  /** Chooses which tags are eligible to be aimed at. */
  public void setTargetFilter(TargetFilter newFilter) {
    if (newFilter != filter) {
      System.out.println("[LIMELIGHT] Target filter -> " + newFilter);
      filter = newFilter;
      goodFrames = 0;
      lockedTarget = Optional.empty();
    }
  }

  public TargetFilter getTargetFilter() {
    return filter;
  }

  /** Pipeline the camera reports running, or -1 if unknown. */
  public int getPipeline() {
    return (int) getPipeEntry.getDouble(-1);
  }

  /** Total measurement age the camera reports, seconds. */
  public double getLatencySeconds() {
    return (tlEntry.getDouble(0.0) + clEntry.getDouble(0.0)) / 1000.0;
  }

  public String getStatus() {
    return status;
  }

  // ---- Loop -----------------------------------------------------------------

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    updateConnection(now);
    enforcePipeline();
    visibleTargets = connected ? readTargets() : List.of();
    updateLock();
    updateStatus();
    publish();
    warnIfStuck(now);

    if (connected && !announcedConnection) {
      announcedConnection = true;
      System.out.println("[LIMELIGHT] '" + name + "' connected." + describeClients());
    }
  }

  /** Heartbeat is the cleanest liveness signal; tv's NT revision covers older firmware. */
  private void updateConnection(double now) {
    long hb = (long) hbEntry.getDouble(Long.MIN_VALUE);
    long tvChange = tvEntry.getLastChange();

    if (hb != lastHeartbeat || tvChange != lastTvChange) {
      lastHeartbeat = hb;
      lastTvChange = tvChange;
      lastPublishSeconds = now;
    }

    connected = tvEntry.exists()
        && lastPublishSeconds >= 0.0
        && (now - lastPublishSeconds) < CONNECTION_TIMEOUT_SECONDS;
  }

  /** Keeps the camera in AprilTag mode so detection cannot be switched off behind our back. */
  private void enforcePipeline() {
    int wanted = LimeLightConstants.APRILTAG_PIPELINE;
    if (wanted >= 0 && connected) {
      pipelineEntry.setNumber(wanted);
    }
  }

  /**
   * Parses rawfiducials, rejecting anything that is not a usable measurement.
   *
   * <p>This is the only place raw camera numbers are touched, and nothing leaves it unvalidated.
   */
  private List<VisionTarget> readTargets() {
    double[] raw = rawFiducialsEntry.getDoubleArray(new double[0]);
    if (raw.length < STRIDE) {
      return List.of();
    }

    List<VisionTarget> found = new ArrayList<>(raw.length / STRIDE);
    for (int i = 0; i + STRIDE - 1 < raw.length; i += STRIDE) {
      int id = (int) raw[i + I_ID];
      double tx = raw[i + I_TX];
      double ty = raw[i + I_TY];
      double area = raw[i + I_TA];

      if (!FieldTags.isFieldTag(id)) {
        // Dropping these silently is how "the camera sees nothing" hides a camera that is seeing
        // plenty, just not tags from this field. Record them so the diagnostics can say so.
        rejectedTagIds.add(id);
        continue;
      }
      seenTagIds.add(id);
      if (!finite(tx) || !finite(ty) || Math.abs(tx) > MAX_ANGLE_DEGREES
          || Math.abs(ty) > MAX_ANGLE_DEGREES) {
        continue;
      }

      RangeResult range = solveRange(id, tx, ty, raw[i + I_DIST_CAMERA]);
      if (range == null) {
        continue;
      }

      found.add(new VisionTarget(id, tx, ty, finite(area) ? area : 0.0,
          range.meters, range.source));
    }

    found.sort((a, b) -> Double.compare(a.rangeMeters, b.rangeMeters));
    return Collections.unmodifiableList(found);
  }

  private static final class RangeResult {
    final double meters;
    final String source;

    RangeResult(double meters, String source) {
      this.meters = meters;
      this.source = source;
    }
  }

  /**
   * Ground range from the lens to a tag.
   *
   * <p>Trigonometry on ty is preferred: the tag's height is known exactly from the field layout,
   * so the range depends only on the camera's own height and pitch. The camera's 3D solve is the
   * fallback. Returns null when neither gives a believable answer, which drops the tag rather than
   * letting a bad range through.
   */
  private RangeResult solveRange(int tagId, double txDegrees, double tyDegrees, double distToCamera) {
    double deltaHeight = FieldTags.heightMeters(tagId) - LimeLightConstants.MOUNT_HEIGHT_METERS;
    double range = CameraGeometry.groundRangeMeters(
        txDegrees, tyDegrees, LimeLightConstants.MOUNT_PITCH_DEGREES, deltaHeight);
    if (plausible(range)) {
      return new RangeResult(range, "TY");
    }

    if (finite(distToCamera) && distToCamera > 0.0) {
      double squared = (distToCamera * distToCamera) - (deltaHeight * deltaHeight);
      double ground = squared > 0.0 ? Math.sqrt(squared) : distToCamera;
      if (plausible(ground)) {
        return new RangeResult(ground, "LL3D");
      }
    }

    return null;
  }

  private static boolean finite(double v) {
    return !Double.isNaN(v) && !Double.isInfinite(v);
  }

  private static boolean plausible(double rangeMeters) {
    return finite(rangeMeters)
        && rangeMeters >= MIN_RANGE_METERS
        && rangeMeters <= MAX_RANGE_METERS;
  }

  /** Debounced acquire and drop, so one odd frame cannot move a mechanism. */
  private void updateLock() {
    Optional<VisionTarget> candidate = pickEligible();

    if (candidate.isPresent()) {
      missedFrames = 0;
      goodFrames = Math.min(goodFrames + 1, FRAMES_TO_ACQUIRE);
      if (goodFrames >= FRAMES_TO_ACQUIRE) {
        lockedTarget = candidate;
      }
      return;
    }

    goodFrames = 0;
    missedFrames++;
    if (missedFrames >= FRAMES_TO_DROP) {
      lockedTarget = Optional.empty();
    }
  }

  private Optional<VisionTarget> pickEligible() {
    if (visibleTargets.isEmpty()) {
      return Optional.empty();
    }
    if (filter == TargetFilter.ANY_TAG) {
      return Optional.of(visibleTargets.get(0));
    }
    Set<Integer> allowed = FieldTags.hubTags();
    return visibleTargets.stream().filter(t -> allowed.contains(t.id)).findFirst();
  }

  private void updateStatus() {
    if (!connected) {
      status = "DISCONNECTED";
    } else if (visibleTargets.isEmpty()) {
      status = "CONNECTED, NO TAGS";
    } else if (lockedTarget.isEmpty()) {
      status = String.format("SEES %d TAG(S), NONE ELIGIBLE (%s)", visibleTargets.size(), filter);
    } else {
      status = "LOCKED ON " + FieldTags.describe(lockedTarget.get().id);
    }
  }

  // ---- Telemetry ------------------------------------------------------------

  private final frc.robot.util.Throttle publishRate =
      new frc.robot.util.Throttle(frc.robot.util.Throttle.DASHBOARD_HZ);

  private void publish() {
    // Logger writes to the AdvantageKit log on disk, which is cheap and worth having at full
    // rate. SmartDashboard goes over the radio, so it is throttled -- these were duplicating the
    // same values to both, at 50 Hz, for no benefit.
    frc.robot.util.RobotLog.put("Limelight/Status", status);
    frc.robot.util.RobotLog.put("Limelight/Connected", connected);
    frc.robot.util.RobotLog.put("Limelight/HasTarget", hasTarget());
    frc.robot.util.RobotLog.put("Limelight/VisibleCount", visibleTargets.size());

    VisionTarget logged = lockedTarget.orElse(null);
    frc.robot.util.RobotLog.put("Limelight/TargetId", logged == null ? -1 : logged.id);
    frc.robot.util.RobotLog.put("Limelight/TargetRange", logged == null ? 0.0 : logged.rangeMeters);

    if (!publishRate.ready()) {
      return;
    }

    SmartDashboard.putString("Limelight/Status", status);
    SmartDashboard.putBoolean("Limelight/Connected", connected);
    SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
    SmartDashboard.putNumber("Limelight/VisibleCount", visibleTargets.size());
    SmartDashboard.putString("Limelight/Visible", describeVisible());
    SmartDashboard.putString("Limelight/Filter", filter.toString());
    SmartDashboard.putNumber("Limelight/Pipeline", getPipeline());

    VisionTarget t = lockedTarget.orElse(null);
    SmartDashboard.putNumber("Limelight/Target/ID", t == null ? -1 : t.id);
    SmartDashboard.putNumber("Limelight/Target/TX", t == null ? 0.0 : t.txDegrees);
    SmartDashboard.putNumber("Limelight/Target/Range", t == null ? 0.0 : t.rangeMeters);
    SmartDashboard.putString("Limelight/Target/RangeSource", t == null ? "NONE" : t.rangeSource);
    SmartDashboard.putNumber("Limelight/Target/RangeFeet", t == null ? 0.0 : t.rangeMeters * 3.28084);

  }

  /** e.g. {@code "20 (BLUE HUB) 2.14m, 1 (RED TRENCH) 3.02m"}. */
  public String describeVisible() {
    if (!connected) {
      return "camera not connected";
    }
    if (visibleTargets.isEmpty()) {
      return "none";
    }
    StringBuilder out = new StringBuilder();
    for (VisionTarget t : visibleTargets) {
      if (out.length() > 0) {
        out.append(", ");
      }
      out.append(FieldTags.describe(t.id)).append(String.format(" %.2fm", t.rangeMeters));
    }
    return out.toString();
  }

  // ---- Diagnostics ----------------------------------------------------------

  private void warnIfStuck(double now) {
    if (lastWarnSeconds >= 0.0 && now - lastWarnSeconds < WARN_PERIOD_SECONDS) {
      return;
    }

    if (!connected) {
      lastWarnSeconds = now;
      DriverStation.reportWarning(
          String.format(
              "[LIMELIGHT] '%s' NOT CONNECTED (%s). Check power and ethernet, and that the "
                  + "camera's hostname is exactly '%s'. NT clients:%s",
              name,
              tvEntry.exists() ? "stopped publishing" : "NT table '" + table.getPath() + "' empty",
              name,
              describeClients()),
          false);
      return;
    }

    if (visibleTargets.isEmpty()) {
      lastWarnSeconds = now;
      DriverStation.reportWarning(
          String.format(
              "[LIMELIGHT] Connected and healthy but detecting NO tags, on pipeline %d. Confirm "
                  + "that pipeline is AprilTag/Fiducial type, family 36h11, and exposure is low "
                  + "enough that the tag is not washed out.",
              getPipeline()),
          false);
      return;
    }

    // Tags are visible but the filter rejects all of them. This is the failure that looks exactly
    // like "the camera and the turret are not talking": detection works, the dashboard shows the
    // tag, and the mechanism never moves because nothing eligible was ever handed to it.
    if (lockedTarget.isEmpty() && goodFrames == 0) {
      lastWarnSeconds = now;
      DriverStation.reportWarning(
          String.format(
              "[LIMELIGHT] Seeing %s, but NONE are eligible under filter %s. "
                  + "%s Switch the filter to ANY_TAG to aim at whatever is in view.",
              describeVisible(),
              filter,
              filter == TargetFilter.ALLIANCE_HUB
                  ? "ALLIANCE_HUB only accepts " + FieldTags.hubTags() + "."
                  : ""),
          false);
    }
  }

  /** Dumps every key the camera publishes. The definitive "what pipeline is this" check. */
  public void dumpDiagnostics() {
    System.out.println("=== LIMELIGHT '" + name + "' DUMP ===");
    reportTagIdCoverage();
    Set<String> keys = table.getKeys();
    if (keys.isEmpty()) {
      System.out.println("  NT table '" + table.getPath() + "' has NO keys.");
      System.out.println("  Not publishing under this name. NT clients:" + describeClients());
      System.out.println("=====================================");
      return;
    }

    List<String> sorted = new ArrayList<>(keys);
    Collections.sort(sorted);
    for (String key : sorted) {
      System.out.printf("  %-22s %s%n", key, describeEntry(table.getEntry(key)));
    }
    System.out.println("  ---");
    System.out.printf("  pipeline             %d%n", getPipeline());
    System.out.printf("  rawfiducials key     %s%n",
        keys.contains("rawfiducials")
            ? "PRESENT"
            : "MISSING  <-- this pipeline is NOT an AprilTag pipeline");
    System.out.printf("  valid tags parsed    %d%n", visibleTargets.size());
    System.out.printf("  visible              %s%n", describeVisible());
    System.out.printf("  filter               %s%n", filter);
    System.out.printf("  locked               %s%n",
        lockedTarget.map(t -> FieldTags.describe(t.id)).orElse("NONE"));
    System.out.println("=====================================");
  }

  /**
   * Back-solves the camera mounting constants from one tape-measured distance.
   *
   * <p>Range comes from {@code range = (tagHeight - cameraHeight) / tan(cameraPitch + ty)}. The
   * tag height is exact, and ty is measured, so a wrong range means {@code MOUNT_HEIGHT_METERS} or
   * {@code MOUNT_PITCH_DEGREES} is wrong — both are currently estimates. Rather than guess at them
   * one redeploy at a time, this prints what each would have to be for the reported range to match
   * the real one.
   *
   * <p>Procedure: park with a tag in view, tape-measure from the LENS to the tag, type that into
   * {@code Limelight/CalibrationDistanceMeters} on the dashboard, and run this. Fix whichever of
   * the two constants you trust less, then re-run to confirm.
   *
   * @param knownRangeMeters true horizontal lens-to-tag distance
   */
  public void printRangeCalibration(double knownRangeMeters) {
    System.out.println("=== LIMELIGHT RANGE CALIBRATION ===");

    Optional<VisionTarget> target = lockedTarget.isPresent()
        ? lockedTarget
        : (visibleTargets.isEmpty() ? Optional.empty() : Optional.of(visibleTargets.get(0)));

    if (target.isEmpty()) {
      System.out.println("  No tag visible. Point the camera at one and try again.");
      System.out.println("===================================");
      return;
    }
    if (!(knownRangeMeters > 0.0)) {
      System.out.println("  Set Limelight/CalibrationDistanceMeters to the measured distance first.");
      System.out.println("===================================");
      return;
    }

    VisionTarget t = target.get();
    double tagHeight = FieldTags.heightMeters(t.id);
    double assumedCameraHeight = LimeLightConstants.MOUNT_HEIGHT_METERS;
    double assumedPitch = LimeLightConstants.MOUNT_PITCH_DEGREES;
    double deltaHeight = tagHeight - assumedCameraHeight;

    // Pitch that would make the current height assumption produce the measured range.
    double requiredPitch = CameraGeometry.calibratedPitchDegrees(
        t.txDegrees, t.tyDegrees, knownRangeMeters, deltaHeight);

    // Camera height that would make the current pitch assumption produce the measured range.
    var ray = CameraGeometry.robotRelativeRay(t.txDegrees, t.tyDegrees, assumedPitch, 0.0);
    double requiredHeight = tagHeight
        - knownRangeMeters * ray.getZ() / Math.hypot(ray.getX(), ray.getY());

    System.out.printf("  tag                  %s%n", FieldTags.describe(t.id));
    System.out.printf("  tag height           %.3f m (%.1f in, exact from field layout)%n",
        tagHeight, tagHeight * 39.3701);
    System.out.printf("  measured ty          %.2f deg%n", t.tyDegrees);
    System.out.printf("  reported range       %.3f m (%.2f ft)  [%s]%n",
        t.rangeMeters, t.rangeMeters * 3.28084, t.rangeSource);
    System.out.printf("  true range           %.3f m (%.2f ft)%n",
        knownRangeMeters, knownRangeMeters * 3.28084);
    System.out.printf("  ERROR                %.3f m (%.2f ft)%n",
        t.rangeMeters - knownRangeMeters, (t.rangeMeters - knownRangeMeters) * 3.28084);
    System.out.println("  ---");
    System.out.printf("  currently assuming   height %.3f m (%.1f in), pitch %.1f deg%n",
        assumedCameraHeight, assumedCameraHeight * 39.3701, assumedPitch);
    System.out.println("  To make the numbers agree, change ONE of these:");
    System.out.printf("    MOUNT_PITCH_DEGREES  = %.2f    (keeping height as-is)%n", requiredPitch);
    System.out.printf("    MOUNT_HEIGHT_METERS  = %.4f  (%.2f in, keeping pitch as-is)%n",
        requiredHeight, requiredHeight * 39.3701);
    System.out.println("  Measure the lens height with a tape and set the OTHER one.");
    System.out.println("===================================");
  }

  /** Runs the range calibration against the distance typed on the dashboard. */
  public Command calibrateRangeCommand() {
    return runOnce(() -> printRangeCalibration(
        SmartDashboard.getNumber("Limelight/CalibrationDistanceMeters", 0.0)))
        .ignoringDisable(true)
        .withName("Limelight.calibrateRange");
  }

  public Command dumpDiagnosticsCommand() {
    return runOnce(this::dumpDiagnostics).ignoringDisable(true).withName("Limelight.dump");
  }

  /** Toggles between aiming at any tag and aiming only at this alliance's hub. */
  public Command toggleTargetFilterCommand() {
    return runOnce(() -> setTargetFilter(
        filter == TargetFilter.ANY_TAG ? TargetFilter.ALLIANCE_HUB : TargetFilter.ANY_TAG))
        .ignoringDisable(true)
        .withName("Limelight.toggleFilter");
  }

  private static String describeEntry(NetworkTableEntry entry) {
    switch (entry.getType()) {
      case kDouble:
        return String.valueOf(entry.getDouble(0.0));
      case kBoolean:
        return String.valueOf(entry.getBoolean(false));
      case kString:
        return entry.getString("");
      case kDoubleArray:
        double[] v = entry.getDoubleArray(new double[0]);
        return v.length == 0 ? "double[0] (empty)"
            : String.format("double[%d] %s", v.length, java.util.Arrays.toString(v));
      default:
        return "(" + entry.getType() + ")";
    }
  }

  /** Attached NT clients, which is how you find the camera's real IP. */
  public static String describeClients() {
    ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
    if (connections.length == 0) {
      return " (none)";
    }
    StringBuilder out = new StringBuilder();
    for (ConnectionInfo c : connections) {
      out.append(String.format("%n    %-22s %s:%d", c.remote_id, c.remote_ip, c.remote_port));
    }
    return out.toString();
  }

  // ---- Network discovery ----------------------------------------------------

  private static boolean forwardingStarted;
  // Leave radio bandwidth available for Driver Station packets and telemetry. This is a
  // payload budget (1.5 Mbit/s), shared across all three routes and both allowed viewers.
  private static final BoundedTcpForwarder.VideoBudget VIDEO_BUDGET =
      new BoundedTcpForwarder.VideoBudget(2, 187_500);

  /** Forwards camera video with backpressure, and web/configuration ports through WPILib. */
  public static synchronized void setupPortForwarding() {
    if (forwardingStarted) {
      return;
    }
    forwardingStarted = true;
    forward(5800, "limelight.local");

    int team = RobotController.getTeamNumber();
    String staticIp = team > 0 ? String.format("10.%d.%d.11", team / 100, team % 100) : null;
    if (staticIp != null) {
      // 5810 belongs to the robot's NT4 server; never bind a camera forward there.
      forward(5830, staticIp);
    }
    forward(5820, "172.29.0.1");

    System.out.println("=== LIMELIGHT: first URL that loads wins ===");
    System.out.println("  http://roborio-" + team + "-frc.local:5801   (mDNS)");
    if (staticIp != null) {
      System.out.printf("  http://roborio-%d-frc.local:5831   (static IP %s)%n", team, staticIp);
    }
    System.out.println("  http://roborio-" + team + "-frc.local:5821   (USB into the camera)");
    System.out.println("===========================================");
  }

  /**
   * Best-guess MJPEG stream URL for the dashboard's camera view.
   *
   * <p>Prefers the address the camera is actually connected from, which is known because the
   * Limelight joins the roboRIO's NetworkTables server as a client. That beats guessing between
   * mDNS, static IP and USB, since which of those is alive changes with how the robot is cabled.
   */
  public static String streamUrlHint() {
    for (ConnectionInfo c : NetworkTableInstance.getDefault().getConnections()) {
      if (c.remote_id != null && c.remote_id.toLowerCase().contains("limelight")) {
        return "http://" + c.remote_ip + ":5800/stream.mjpg";
      }
    }
    int team = RobotController.getTeamNumber();
    if (team > 0) {
      return String.format("http://10.%d.%d.11:5800/stream.mjpg", team / 100, team % 100);
    }
    return "http://limelight.local:5800/stream.mjpg";
  }

  private static void forward(int localBase, String host) {
    try {
      // WPILib 2026.2.1's native forwarder queues writes without backpressure. A USB MJPEG
      // stream can exhaust roboRIO memory when the radio/viewer cannot keep up.
      new BoundedTcpForwarder(localBase, host, 5800, VIDEO_BUDGET);
    } catch (java.io.IOException e) {
      DriverStation.reportWarning("Could not forward Limelight video on " + localBase
          + ": " + e.getMessage(), false);
    }
    for (int i = 1; i < 10; i++) {
      PortForwarder.add(localBase + i, host, 5800 + i);
    }
  }
}
