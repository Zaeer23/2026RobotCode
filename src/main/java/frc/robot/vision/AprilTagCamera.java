package frc.robot.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Reads AprilTags off a Limelight over NetworkTables.
 *
 * <h2>Design rules</h2>
 *
 * <ol>
 *   <li><b>Report everything, filter nowhere.</b> Every tag the camera reports is published, with
 *       no alliance or ID filtering anywhere in this class. Callers filter if they want to. The
 *       previous version pushed an ID filter onto the camera itself, which persisted in hardware
 *       and made a perfectly healthy camera look blind.
 *   <li><b>Never touch camera configuration.</b> No pipeline switching, no LED commands. Config
 *       belongs in the Limelight web UI. Code that "helpfully" selects a pipeline at boot is code
 *       that can silently move a working camera onto an empty one.
 *   <li><b>Say which failure it is.</b> "No target" has several causes needing different fixes, so
 *       this distinguishes disconnected / connected-but-blind / seeing-unexpected-tags and says so.
 * </ol>
 *
 * <p>Everything lands on SmartDashboard under {@code AprilTag/} and in the AdvantageKit log, so the
 * question "can we see tags" is answerable without a debugger.
 */
public class AprilTagCamera extends SubsystemBase {

  /** Layout of the {@code rawfiducials} array: 7 doubles per tag. */
  private static final int STRIDE = 7;
  private static final int IDX_ID = 0;
  private static final int IDX_TXNC = 1;
  private static final int IDX_TYNC = 2;
  private static final int IDX_TA = 3;
  private static final int IDX_DIST_TO_CAMERA = 4;
  private static final int IDX_DIST_TO_ROBOT = 5;
  private static final int IDX_AMBIGUITY = 6;

  /** No publish for this long means the camera is gone, not merely looking at a blank wall. */
  private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;

  /** Gap between repeats of the same driver-station complaint. */
  private static final double WARNING_PERIOD_SECONDS = 5.0;

  private final String name;
  private final NetworkTable table;
  private final NetworkTableEntry tv;
  private final NetworkTableEntry tl;
  private final NetworkTableEntry cl;
  private final NetworkTableEntry heartbeat;
  private final NetworkTableEntry pipelineIndex;
  private final NetworkTableEntry rawFiducials;

  private List<TagObservation> tags = List.of();
  private boolean connected = false;
  private double lastPublishSeconds = -1.0;
  private long lastHeartbeat = -1L;
  private long lastTvChange = -1L;
  private double lastWarningSeconds = -1.0;
  private String status = "STARTING";
  private boolean announcedAddress = false;

  /**
   * @param name NetworkTables name of the camera, which is its hostname in the web UI. A mismatch
   *             here is indistinguishable from a dead camera, so it is reported explicitly.
   */
  public AprilTagCamera(String name) {
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.tv = table.getEntry("tv");
    this.tl = table.getEntry("tl");
    this.cl = table.getEntry("cl");
    this.heartbeat = table.getEntry("hb");
    this.pipelineIndex = table.getEntry("getpipe");
    this.rawFiducials = table.getEntry("rawfiducials");
    setName("AprilTagCamera[" + name + "]");
  }

  // ---- Public API -----------------------------------------------------------

  /** True when the camera is publishing to NetworkTables right now. */
  public boolean isConnected() {
    return connected;
  }

  /** Every tag currently visible, nearest first. Empty when none or disconnected. */
  public List<TagObservation> getTags() {
    return tags;
  }

  /** True when at least one tag is visible. */
  public boolean hasAnyTag() {
    return !tags.isEmpty();
  }

  /** The nearest visible tag, whatever it is. */
  public Optional<TagObservation> getBestTag() {
    return tags.isEmpty() ? Optional.empty() : Optional.of(tags.get(0));
  }

  /** The nearest visible tag whose ID is in {@code allowed}. */
  public Optional<TagObservation> getBestTag(Set<Integer> allowed) {
    return tags.stream().filter(t -> allowed.contains(t.id)).findFirst();
  }

  /** The nearest visible tag on the given structure. */
  public Optional<TagObservation> getBestTag(TagFunction function) {
    return tags.stream().filter(t -> t.function == function).findFirst();
  }

  /** The nearest visible HUB tag belonging to this alliance. */
  public Optional<TagObservation> getBestHubTag() {
    return getBestTag(FieldTags.hubTags());
  }

  /** IDs of every visible tag, for logging and diagnostics. */
  public int[] getVisibleTagIds() {
    int[] ids = new int[tags.size()];
    for (int i = 0; i < tags.size(); i++) {
      ids[i] = tags.get(i).id;
    }
    return ids;
  }

  /** Pipeline the camera reports itself to be running, or -1 if unknown. */
  public int getPipelineIndex() {
    return (int) pipelineIndex.getDouble(-1);
  }

  /** Total measurement age reported by the camera, in seconds. */
  public double getLatencySeconds() {
    return (tl.getDouble(0.0) + cl.getDouble(0.0)) / 1000.0;
  }

  /** One-line summary of the camera's state, suitable for a dashboard string. */
  public String getStatus() {
    return status;
  }

  // ---- Loop -----------------------------------------------------------------

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    updateConnection(now);
    tags = connected ? readTags() : List.of();
    updateStatus();
    publishTelemetry();
    reportProblems(now);
    announceAddressOnce();
  }

  /**
   * Tracks whether the camera is alive.
   *
   * <p>Limelight bumps {@code hb} every frame, which is the cleanest liveness signal. Older
   * firmware may not publish it, so a change in {@code tv}'s NT revision is used as a fallback —
   * between them, any publishing camera is detected.
   */
  private void updateConnection(double now) {
    long hb = (long) heartbeat.getDouble(-1);
    long tvChange = tv.getLastChange();

    boolean published = (hb >= 0 && hb != lastHeartbeat) || (tvChange != lastTvChange);
    if (published) {
      lastHeartbeat = hb;
      lastTvChange = tvChange;
      lastPublishSeconds = now;
    }

    connected =
        table.getEntry("tv").exists()
            && lastPublishSeconds >= 0.0
            && (now - lastPublishSeconds) < CONNECTION_TIMEOUT_SECONDS;
  }

  /** Parses {@code rawfiducials} into observations, nearest first. No filtering of any kind. */
  private List<TagObservation> readTags() {
    double[] raw = rawFiducials.getDoubleArray(new double[0]);
    if (raw.length < STRIDE) {
      return List.of();
    }

    List<TagObservation> found = new ArrayList<>(raw.length / STRIDE);
    for (int i = 0; i + STRIDE - 1 < raw.length; i += STRIDE) {
      int id = (int) raw[i + IDX_ID];
      double distToCamera = raw[i + IDX_DIST_TO_CAMERA];
      double distToRobot = raw[i + IDX_DIST_TO_ROBOT];

      found.add(TagObservation.of(
          id,
          raw[i + IDX_TXNC],
          raw[i + IDX_TYNC],
          raw[i + IDX_TA],
          Double.isFinite(distToCamera) ? distToCamera : 0.0,
          Double.isFinite(distToRobot) ? distToRobot : 0.0,
          raw[i + IDX_AMBIGUITY]));
    }

    // Nearest first, falling back to largest-in-frame when the 3D solve gives nothing.
    found.sort((a, b) -> {
      boolean aHasRange = a.distanceToCameraMeters > 0.0;
      boolean bHasRange = b.distanceToCameraMeters > 0.0;
      if (aHasRange && bHasRange) {
        return Double.compare(a.distanceToCameraMeters, b.distanceToCameraMeters);
      }
      return Double.compare(b.areaPercent, a.areaPercent);
    });
    return Collections.unmodifiableList(found);
  }

  private void updateStatus() {
    if (!connected) {
      status = "DISCONNECTED";
    } else if (tags.isEmpty()) {
      status = "CONNECTED, NO TAGS";
    } else {
      status = String.format("SEEING %d TAG(S)", tags.size());
    }
  }

  // ---- Telemetry ------------------------------------------------------------

  private void publishTelemetry() {
    SmartDashboard.putString("AprilTag/Status", status);
    SmartDashboard.putBoolean("AprilTag/Connected", connected);
    SmartDashboard.putNumber("AprilTag/TagCount", tags.size());
    SmartDashboard.putNumber("AprilTag/Pipeline", getPipelineIndex());
    SmartDashboard.putNumber("AprilTag/LatencySeconds", getLatencySeconds());
    SmartDashboard.putString("AprilTag/VisibleTags", describeVisibleTags());
    SmartDashboard.putString(
        "AprilTag/Alliance", FieldTags.alliance().map(Enum::name).orElse("UNKNOWN"));

    Optional<TagObservation> best = getBestTag();
    SmartDashboard.putNumber("AprilTag/Best/ID", best.map(t -> t.id).orElse(-1));
    SmartDashboard.putString(
        "AprilTag/Best/Function", best.map(t -> t.function.name()).orElse("NONE"));
    SmartDashboard.putNumber("AprilTag/Best/TX", best.map(t -> t.txDegrees).orElse(0.0));
    SmartDashboard.putNumber("AprilTag/Best/TY", best.map(t -> t.tyDegrees).orElse(0.0));
    SmartDashboard.putNumber(
        "AprilTag/Best/DistanceMeters", best.map(t -> t.distanceToCameraMeters).orElse(0.0));

    Optional<TagObservation> hub = getBestHubTag();
    SmartDashboard.putBoolean("AprilTag/Hub/Visible", hub.isPresent());
    SmartDashboard.putNumber("AprilTag/Hub/ID", hub.map(t -> t.id).orElse(-1));
    SmartDashboard.putNumber(
        "AprilTag/Hub/DistanceMeters", hub.map(t -> t.distanceToCameraMeters).orElse(0.0));

    Logger.recordOutput("AprilTag/Connected", connected);
    Logger.recordOutput("AprilTag/Status", status);
    Logger.recordOutput("AprilTag/TagCount", tags.size());
    Logger.recordOutput("AprilTag/VisibleTagIds", getVisibleTagIds());
    Logger.recordOutput("AprilTag/Pipeline", getPipelineIndex());
    Logger.recordOutput("AprilTag/LatencySeconds", getLatencySeconds());
  }

  /** e.g. {@code "20 (BLUE HUB) 2.14m, 19 (BLUE HUB) 2.31m"}, or a reason when empty. */
  public String describeVisibleTags() {
    if (!connected) {
      return "camera not connected";
    }
    if (tags.isEmpty()) {
      return "none";
    }
    StringBuilder out = new StringBuilder();
    for (TagObservation tag : tags) {
      if (out.length() > 0) {
        out.append(", ");
      }
      out.append(String.format(
          "%s %.2fm", FieldTags.describe(tag.id), tag.distanceToCameraMeters));
    }
    return out.toString();
  }

  // ---- Diagnostics ----------------------------------------------------------

  private void reportProblems(double now) {
    if (lastWarningSeconds >= 0.0 && now - lastWarningSeconds < WARNING_PERIOD_SECONDS) {
      return;
    }

    if (!connected) {
      lastWarningSeconds = now;
      String detail = table.getEntry("tv").exists()
          ? String.format("stopped publishing %.1fs ago", now - lastPublishSeconds)
          : "has never published; NT table '" + table.getPath() + "' is empty";
      DriverStation.reportWarning(
          String.format(
              "[APRILTAG] Camera '%s' NOT CONNECTED (%s). Check power and ethernet, and confirm "
                  + "the camera's hostname is exactly '%s'. Attached NT clients:%s",
              name, detail, name, describeNetworkTablesClients()),
          false);
      return;
    }

    if (tags.isEmpty()) {
      lastWarningSeconds = now;
      DriverStation.reportWarning(
          String.format(
              "[APRILTAG] Camera '%s' is connected and healthy but detects NO tags. It is running "
                  + "pipeline %d — confirm that pipeline is an AprilTag/Fiducial type with family "
                  + "36h11, and check exposure is low enough that the tag is not washed out.",
              name, getPipelineIndex()),
          false);
      return;
    }

    // Seeing tags, but none belong to the 2026 field: almost always a stale printed practice tag.
    boolean anyFieldTag = tags.stream().anyMatch(TagObservation::isFieldTag);
    if (!anyFieldTag) {
      lastWarningSeconds = now;
      DriverStation.reportWarning(
          String.format(
              "[APRILTAG] Seeing tag(s) %s, but none are 2026 REBUILT field tags (valid IDs are "
                  + "1-32). Check the tag family is 36h11 and the printed tags are the 2026 set.",
              describeVisibleTags()),
          false);
    }
  }

  private void announceAddressOnce() {
    if (announcedAddress || !connected) {
      return;
    }
    announcedAddress = true;
    System.out.println(
        "[APRILTAG] Camera '" + name + "' connected. NT clients attached:"
            + describeNetworkTablesClients());
    // Dumped on first connection rather than at boot, because at boot the camera has not published
    // anything yet and the key list would be empty and misleading.
    dumpNetworkTablesKeys();
  }

  /** Prints the NT dump on demand. Bind to a button when debugging at the field. */
  public edu.wpi.first.wpilibj2.command.Command dumpDiagnosticsCommand() {
    return runOnce(this::dumpNetworkTablesKeys)
        .ignoringDisable(true)
        .withName("AprilTag.dumpDiagnostics");
  }

  /**
   * Dumps every key the camera publishes, with its current value.
   *
   * <p>This is the diagnostic that settles "why does it see no tags" without guessing, because the
   * key set tells you what kind of pipeline is running:
   *
   * <ul>
   *   <li>{@code rawfiducials} present and non-empty -> tags are being detected; the problem is
   *       downstream of the camera.
   *   <li>{@code rawfiducials} present but always empty -> an AprilTag pipeline that is detecting
   *       nothing. Look at exposure, tag family (must be 36h11) and focus.
   *   <li>{@code rawfiducials} MISSING entirely -> the active pipeline is not an AprilTag pipeline
   *       at all. No amount of exposure tuning will help; change the pipeline type in the web UI.
   *   <li>{@code tclass} or {@code tcornxy} present -> a classifier or retroreflective pipeline.
   * </ul>
   *
   * <p>Printed once at boot and on demand, not every loop.
   */
  public void dumpNetworkTablesKeys() {
    System.out.println("=== APRILTAG CAMERA '" + name + "' NT DUMP ===");
    Set<String> keys = table.getKeys();
    if (keys.isEmpty()) {
      System.out.println("  NT table '" + table.getPath() + "' has NO keys.");
      System.out.println("  The camera is not publishing under this name. Either it is offline, or");
      System.out.println("  its hostname is not '" + name + "'. Attached NT clients:"
          + describeNetworkTablesClients());
      System.out.println("===========================================");
      return;
    }

    List<String> sorted = new ArrayList<>(keys);
    Collections.sort(sorted);
    for (String key : sorted) {
      System.out.printf("  %-22s %s%n", key, describeEntry(table.getEntry(key)));
    }

    boolean hasFiducials = keys.contains("rawfiducials");
    System.out.println("  ---");
    System.out.printf("  active pipeline      %d%n", getPipelineIndex());
    System.out.printf("  rawfiducials key     %s%n",
        hasFiducials ? "PRESENT" : "MISSING  <-- pipeline is probably NOT an AprilTag pipeline");
    System.out.printf("  tags decoded now     %d%n", tags.size());
    System.out.println("===========================================");
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
        double[] values = entry.getDoubleArray(new double[0]);
        return values.length == 0
            ? "double[0]  (empty)"
            : String.format("double[%d] %s", values.length, java.util.Arrays.toString(values));
      default:
        return "(" + entry.getType() + ")";
    }
  }

  /** Addresses of all attached NT clients, which is how you find the camera's real IP. */
  public static String describeNetworkTablesClients() {
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

  /**
   * Forwards the camera's web ports through the roboRIO over every route at once.
   *
   * <p>A PortForwarder entry is keyed by LOCAL port, so each route needs its own block; pointing
   * two routes at the same local ports makes the second silently replace the first. Three are set
   * up because each fails differently: mDNS depends on flaky name resolution, the static IP needs
   * the camera to actually be on the team network, and USB works regardless of both.
   */
  public static void setupPortForwarding() {
    forwardRange(5800, "limelight.local");

    int team = RobotController.getTeamNumber();
    String staticIp = team > 0 ? String.format("10.%d.%d.11", team / 100, team % 100) : null;
    if (staticIp != null) {
      forwardRange(5810, staticIp);
    }

    forwardRange(5820, "172.29.0.1");

    System.out.println("=== APRILTAG CAMERA: try these, first one that loads wins ===");
    System.out.println("  http://localhost:5801   (via limelight.local / mDNS)");
    if (staticIp != null) {
      System.out.printf("  http://localhost:5811   (via static IP %s)%n", staticIp);
    }
    System.out.println("  http://localhost:5821   (via USB cable into the camera)");
    System.out.println("============================================================");
  }

  private static void forwardRange(int localBasePort, String host) {
    for (int i = 0; i < 10; i++) {
      PortForwarder.add(localBasePort + i, host, 5800 + i);
    }
  }
}
