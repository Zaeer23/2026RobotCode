package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

/*

  File Destination:    frc/robot/subsystems/vision/LimeLight.java

  Command Execution:  frc/robot/commands/vision/LimeLightRunner.java

*/
public class LimeLight {

  public static double ALIGN_KP = 0.06;
  public static double ALIGN_MAX_OMEGA_RAD_S = 1.5;
  public static double ALIGN_TOLERANCE_DEGREES = 1.0;
  public static double CLIMB_ALIGN_FORWARD_KP = 0.35;
  public static double CLIMB_ALIGN_MAX_FORWARD = 0.45;
  public static double CLIMB_ALIGN_CLOSE_MAX_FORWARD = 0.18;
  public static double CLIMB_ALIGN_DESIRED_AREA = 2.0;
  public static double CLIMB_ALIGN_AREA_TOLERANCE = 0.20;
  public static double CLIMB_ALIGN_SLOW_AREA_ERROR = 0.60;
  public static double CLIMB_ALIGN_TURN_ONLY_ERROR_DEGREES = 6.0;
  public static double CLIMB_ALIGN_TX_FILTER_ALPHA = 0.35;
  public static double CLIMB_ALIGN_AREA_FILTER_ALPHA = 0.25;
  public static double CLIMB_ALIGN_MAX_TX_JUMP_DEGREES = 18.0;
  public static double CLIMB_ALIGN_MAX_FORWARD_STEP = 0.08;
  public static double CLIMB_ALIGN_LATENCY_FUDGE_MS = 20.0;
  public static int CLIMB_ALIGN_JUMP_HOLD_CYCLES = 3;

  // Physical offset of the limelight lens from the robot's true center (inches).
  // Positive = limelight is to the RIGHT of center when viewed from above.
  // Measure from center of robot to center of limelight lens, left/right axis only.
  public static final double LIMELIGHT_LATERAL_OFFSET_INCHES = 9.0; // tune: measure physically

    private static final double APRILTAG_DISTANCE_SCALE = 4202.278;

    private final NetworkTable table;
    private final NetworkTableEntry tvEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tidEntry;
    private final NetworkTableEntry tlEntry;
    private final NetworkTableEntry clEntry;
    private final NetworkTableEntry rawFiducialsEntry;
    private final NetworkTableEntry ledModeEntry;
    private double filteredClimbTxDegrees = 0.0;
    private double filteredClimbArea = 0.0;
    private double lastRawClimbTxDegrees = 0.0;
    private double lastClimbForward = 0.0;
    private boolean hasClimbAlignSample = false;
    private int climbJumpHoldCyclesRemaining = 0;
    private static final double SCAN_CACHE_PERIOD_SECONDS = 0.02;
    private static final int RAW_FIDUCIAL_STRIDE = 7;
    private static final int RAW_FIDUCIAL_ID_INDEX = 0;
    private static final int RAW_FIDUCIAL_TX_INDEX = 1;
    private static final int RAW_FIDUCIAL_TY_INDEX = 2;
    private static final int RAW_FIDUCIAL_TA_INDEX = 3;
    private static final int RAW_FIDUCIAL_DISTANCE_INDEX = 4;
    private static final double MIN_REASONABLE_TAG_DISTANCE_METERS = 0.15;
    private static final double MAX_REASONABLE_TAG_DISTANCE_METERS = 20.0;
    /** NT change-counter of tv the last time we looked, used to detect the camera publishing. */
    private long lastNtChangeMicros = -1L;

    /** FPGA time we last saw the camera publish anything at all. */
    private double lastCameraUpdateSeconds = -1.0;

    private double lastConnectionWarningSeconds = -1.0;

    /** A camera that has not published for this long is treated as gone. */
    private static final double CONNECTION_TIMEOUT_SECONDS = 1.0;

    private AprilTagScan cachedAnyScan = new AprilTagScan(false, -1, 0.0, 0.0, 0.0, 0.0, new Pose3d());
    private double cachedTa = 0.0;
    private double cachedTv = 0.0;
    private double[] cachedRawFiducials = new double[0];
    private double lastScanCacheTimestampSeconds = -1.0;

    public LimeLight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
        tvEntry = table.getEntry("tv");
        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        taEntry = table.getEntry("ta");
        tidEntry = table.getEntry("tid");
        tlEntry = table.getEntry("tl");
        clEntry = table.getEntry("cl");
        rawFiducialsEntry = table.getEntry("rawfiducials");
        ledModeEntry = table.getEntry("ledMode");
    }

    //some bic data access thingymabobies

    public boolean hasTarget() {
        refreshScanCacheIfNeeded();
        return cachedTv == 1;
    }

    /**
     * True when the camera is actually publishing to NetworkTables.
     *
     * <p>Distinct from {@link #hasTarget()}: a connected camera staring at a blank wall has no
     * target, while a disconnected one also has no target. Only this tells the two apart, and they
     * need completely different fixes — aim the camera versus fix the network.
     */
    public boolean isConnected() {
        return tvEntry.exists()
            && lastCameraUpdateSeconds >= 0.0
            && (Timer.getFPGATimestamp() - lastCameraUpdateSeconds) < CONNECTION_TIMEOUT_SECONDS;
    }

    /** Seconds since the camera last published, or -1 if it never has. */
    public double getSecondsSinceLastUpdate() {
        if (lastCameraUpdateSeconds < 0.0) {
            return -1.0;
        }
        return Timer.getFPGATimestamp() - lastCameraUpdateSeconds;
    }

    /**
     * Reports the IP address of every NetworkTables client attached to the robot.
     *
     * <p>The Limelight joins the roboRIO's NT server as a client, so the robot already knows the
     * camera's real address even when nothing else on the network can find it. That is the piece
     * normally missing when the Hardware Manager comes up empty: not "is it there", but "what
     * address is it on". Point a browser or the Hardware Manager straight at whatever this prints.
     */
    public static String describeNetworkTablesClients() {
        ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
        if (connections.length == 0) {
            return "no NetworkTables clients connected";
        }

        StringBuilder out = new StringBuilder();
        for (ConnectionInfo connection : connections) {
            boolean looksLikeLimelight =
                connection.remote_id != null
                    && connection.remote_id.toLowerCase().contains("limelight");
            out.append(String.format(
                "%n    %-22s %s:%d%s",
                connection.remote_id,
                connection.remote_ip,
                connection.remote_port,
                looksLikeLimelight ? "   <-- LIMELIGHT" : ""));
        }
        return out.toString();
    }

    /** Prints the address of every attached NT client, so the camera can be found by IP. */
    public static void reportDiscoveredAddresses() {
        System.out.println(
            "=== LIMELIGHT: NetworkTables clients attached to the robot ==="
                + describeNetworkTablesClients());
        System.out.println(
            "    Point a browser at http://<that ip>:5801, or type that IP into the"
                + " Limelight Hardware Manager.");
        System.out.println("=============================================================");
    }

    private boolean printedConnectedAddress = false;

    /**
     * Prints the camera's address the first time it actually connects.
     *
     * <p>Not done at boot: NetworkTables clients have not attached yet that early, so the list
     * would always come back empty. Waiting for the first real connection means the line that gets
     * printed is the address the camera is genuinely reachable on right now.
     */
    private void reportAddressOnceConnected(double nowSeconds) {
        if (printedConnectedAddress || !isConnected()) {
            return;
        }
        printedConnectedAddress = true;
        reportDiscoveredAddresses();
    }

    private void reportConnectionIfDown(double nowSeconds) {
        if (isConnected()) {
            return;
        }
        if (lastConnectionWarningSeconds >= 0.0 && nowSeconds - lastConnectionWarningSeconds < 5.0) {
            return;
        }
        lastConnectionWarningSeconds = nowSeconds;

        String detail = tvEntry.exists()
            ? String.format("stopped publishing %.1fs ago", getSecondsSinceLastUpdate())
            : "has never published (NT table '" + table.getPath() + "' is empty)";
        DriverStation.reportWarning(
            "[VISION] Limelight NOT CONNECTED: " + detail
                + ". Check power and ethernet, confirm the camera's NT name matches \""
                + frc.robot.Constants.LimeLightConstants.NAME
                + "\", and try the URLs printed at boot.",
            false);
        // Whatever IS attached is the best clue to where the camera actually is.
        System.out.println(
            "[VISION] NetworkTables clients currently attached:"
                + describeNetworkTablesClients());
    }

    public double getTX() {
        return scan().tx;
    }

    public double getTY() {
        return scan().ty;
    }

    public double getTA() {
        refreshScanCacheIfNeeded();
        return cachedTa;
    }

    public int getTagID() {
        return scan().tagID;
    }

    public void lightMode(int number) {
        ledModeEntry.setNumber(number);
    }

    /**
     * Puts the camera into a known-good state at boot.
     *
     * <p><b>Always clears the fiducial ID filter.</b> That filter is CAMERA-side state: it is set
     * over NetworkTables and the Limelight keeps honouring it until something clears it. If a
     * tracking command ever set it and did not get to run its end() — robot disabled mid-command,
     * code redeployed, brownout — the camera silently keeps reporting only those tag IDs and looks
     * completely blind to every other tag, across reboots. Clearing on every boot makes that
     * unrecoverable state impossible.
     *
     * <p><b>Only touches the pipeline when explicitly told to.</b> Forcing an index here is a good
     * way to switch a working camera onto an empty pipeline, so
     * {@link frc.robot.Constants.LimeLightConstants#APRILTAG_PIPELINE} defaults to -1 meaning
     * "leave whatever the camera is already on".
     *
     * <p>The lens pose is deliberately NOT pushed from here. It belongs in the Limelight web UI, so
     * the camera's own 3D solve and Constants stay in sync from a single source and a sign mistake
     * in code cannot silently corrupt the camera output.
     */
    public void configureForAprilTags() {
        clearTagFilter();

        int pipeline = frc.robot.Constants.LimeLightConstants.APRILTAG_PIPELINE;
        if (pipeline >= 0) {
            table.getEntry("pipeline").setNumber(pipeline);
        }
        ledModeEntry.setNumber(0);
    }

    /**
     * Every fiducial ID the camera can currently see, ignoring any allowed-tag set we apply.
     *
     * <p>This is the difference between "the camera is blind" and "the camera sees tags, but our
     * filter rejected them" — two problems with completely different fixes.
     */
    public int[] getVisibleTagIds() {
        refreshScanCacheIfNeeded();
        int count = cachedRawFiducials.length / RAW_FIDUCIAL_STRIDE;
        int[] ids = new int[count];
        for (int i = 0; i < count; i++) {
            ids[i] = (int) cachedRawFiducials[(i * RAW_FIDUCIAL_STRIDE) + RAW_FIDUCIAL_ID_INDEX];
        }
        return ids;
    }

    public double getPipelineLatencyMs() {
        return tlEntry.getDouble(0.0);
    }

    public double getCaptureLatencyMs() {
        return clEntry.getDouble(0.0);
    }

    public double getTotalLatencyMs() {
        return getPipelineLatencyMs() + getCaptureLatencyMs();
    }
    // this is just optional helpers

    // Example forward correction using area
    public double getForwardCorrection(double desiredArea, double kP) {
        return (desiredArea - getTA()) * kP;
    }

    // Shane NEED TO CHANGE MATH, currently require height values, must make the math flexible based on concurrent data.
  // If possible, record limelight position in the future. Use (scale / ta) for simplier distance calculations if needed.

  // example: (return 30666 / limelight.getTA())

  // done! just need to test it, builders pls finish robot 🥺🥺
 
  /*
  12.236 at 20cm
  3.192% at 40cm
  2.025% at 50cm

  pow curve = y = 4202.278x^2 - 1.949067
   */

  public double getLimelightAprilDistance_BasedScales()
  {
    return calculateDistanceFromArea(getTA());
  }

  private double calculateDistanceFromArea(double area) {
    if (area <= 1e-6) {
      return 0.0;
    }
    return APRILTAG_DISTANCE_SCALE / area;
  }

  private Pose3d calculatePoseFromAngles(double txDegrees, double tyDegrees, double distance) {
    double tx = Math.toRadians(txDegrees);
    double ty = Math.toRadians(tyDegrees);

    double x = distance * Math.sin(tx);
    double y = distance * Math.sin(ty);
    double z = distance * Math.cos(ty);

    return new Pose3d(new Translation3d(x, y, z), new Rotation3d());
  }
  public boolean isCentered() {
        return hasTarget() && Math.abs(getCorrectedTX()) < ALIGN_TOLERANCE_DEGREES;
  }

  /**
   * Returns TX corrected for the limelight's lateral offset from robot center.
   * When correctedTX == 0, the robot center (not the limelight) is aimed at the target.
   *
   * correctedTX = tx + atan2(lateralOffset, distance)
   *
   * <p>The parallax term is ADDED, not subtracted. The lens sits to the RIGHT of robot center, so
   * a target sitting dead ahead of the lens (tx = 0) is genuinely off to the robot's right, and a
   * centered camera would read it as positive tx. Compensating therefore means adding.
   *
   * <p>This previously subtracted, which pushed the chassis the wrong way by about 4.5 degrees at
   * 3 m — over four times ALIGN_TOLERANCE_DEGREES, so the align loop settled off-target and the
   * error grew as the robot got closer.
   */
  public double getCorrectedTX() {
    if (!hasTarget()) return 0.0;
    double distanceInches = Math.max(getLimelightAprilDistance_BasedScales() * 39.37, 1.0);
    double parallax = Math.toDegrees(
        Math.atan2(LIMELIGHT_LATERAL_OFFSET_INCHES, distanceInches));
    return getTX() + parallax;
  }

  public double getCorrectedTX(AprilTagScan scan) {
    if (!scan.isValid()) {
      return 0.0;
    }

    double distanceInches = Math.max(scan.distance * 39.37, 1.0);
    double parallax = Math.toDegrees(
        Math.atan2(LIMELIGHT_LATERAL_OFFSET_INCHES, distanceInches));
    return scan.tx + parallax;
  }

  // ---- Solved observations -------------------------------------------------
  // Raw tx/ty are camera-frame angles. Everything that aims or ranges should go through
  // VisionTargeting instead, which resolves them into robot-frame geometry exactly once.

  /** Resolves the closest alliance HUB tag into robot-frame geometry. */
  public VisionTargeting.TargetObservation observeHub() {
    return VisionTargeting.solve(scan(frc.robot.FieldConstants.hubTagIds()), 0.0);
  }

  /**
   * Resolves the closest alliance HUB tag, compensating for how far the chassis has yawed since
   * the frame was captured.
   */
  public VisionTargeting.TargetObservation observeHub(SwerveSubsystem drivebase) {
    double omega = drivebase == null ? 0.0 : drivebase.getRobotVelocity().omegaRadiansPerSecond;
    return VisionTargeting.solve(scan(frc.robot.FieldConstants.hubTagIds()), omega);
  }

  /** Resolves an arbitrary allowed-tag set into robot-frame geometry. */
  public VisionTargeting.TargetObservation observe(
      Set<Integer> allowedTagIds, SwerveSubsystem drivebase) {
    double omega = drivebase == null ? 0.0 : drivebase.getRobotVelocity().omegaRadiansPerSecond;
    return VisionTargeting.solve(scan(allowedTagIds), omega);
  }

  /**
   * Tells the camera to ignore every tag outside this set, so a stray tag in the background cannot
   * become the "closest" target mid-shot.
   */
  public void setTagFilter(Set<Integer> allowedTagIds) {
    double[] ids = new double[allowedTagIds.size()];
    int i = 0;
    for (int id : allowedTagIds) {
      ids[i++] = id;
    }
    table.getEntry("fiducial_id_filters_set").setDoubleArray(ids);
  }

  /** Clears any tag ID filter. */
  public void clearTagFilter() {
    table.getEntry("fiducial_id_filters_set").setDoubleArray(new double[0]);
  }

  public double getAlignOmega() {
        if (!hasTarget()) return 0.0;
        // Use corrected TX so we align robot center, not the limelight, to the target
        double omega = -getCorrectedTX() * ALIGN_KP;
        return Math.max(-ALIGN_MAX_OMEGA_RAD_S, Math.min(ALIGN_MAX_OMEGA_RAD_S, omega));
    }

  // commands

  // ---- Network discovery ---------------------------------------------------
  // A PortForwarder entry is keyed by LOCAL port, so each route needs its own block of local
  // ports. Forwarding several routes at once costs nothing and means you do not have to guess
  // which one is alive: try them in order and the first that loads is your working path.

  /** Local port block forwarding to limelight.local (mDNS). Web UI at localhost:5801. */
  public static final int PORT_BASE_MDNS = 5800;

  /** Local port block forwarding to the team static IP. Web UI at localhost:5811. */
  public static final int PORT_BASE_STATIC_IP = 5810;

  /** Local port block forwarding over USB-ethernet. Web UI at localhost:5821. */
  public static final int PORT_BASE_USB = 5820;

  /**
   * Makes the camera reachable from a laptop tethered to the roboRIO, over every path at once.
   *
   * <p>Three routes are forwarded simultaneously because each fails in a different, common way:
   *
   * <ul>
   *   <li><b>mDNS</b> ({@code limelight.local}) — the usual advice, but roboRIO mDNS resolution is
   *       unreliable, and if the name does not resolve the forward is silently dead.
   *   <li><b>Static IP</b> ({@code 10.TE.AM.11}) — works whenever mDNS does not, which in practice
   *       is most of the time. Requires the camera to actually be set to that address.
   *   <li><b>USB-ethernet</b> ({@code 172.29.0.1}) — works with a cable straight into the camera,
   *       independent of the radio entirely.
   * </ul>
   *
   * <p>This replaces calling setupPortForwardingUSB() and setupPortForwardingRobotWifi()
   * separately, which both claimed local ports 5800-5809 and so silently overwrote each other.
   */
  public static void setupPortForwarding() {
    forwardRange(PORT_BASE_MDNS, "limelight.local");

    String staticIp = staticIpForTeam(RobotController.getTeamNumber());
    if (staticIp != null) {
      forwardRange(PORT_BASE_STATIC_IP, staticIp);
    }

    forwardRange(PORT_BASE_USB, "172.29.0.1");

    System.out.println("=== LIMELIGHT: try these in a browser, first one that loads wins ===");
    System.out.println("  http://localhost:5801   (via limelight.local / mDNS)");
    if (staticIp != null) {
      System.out.printf("  http://localhost:5811   (via static IP %s)%n", staticIp);
    }
    System.out.println("  http://localhost:5821   (via USB cable into the Limelight)");
    System.out.println("  http://limelight.local:5801   (direct, no roboRIO involved)");
    if (staticIp != null) {
      System.out.printf("  http://%s:5801   (direct by IP)%n", staticIp);
    }
    System.out.println("====================================================================");
  }

  /**
   * Standard FRC static address for a device on the team network: 10.TE.AM.11.
   *
   * @return the address, or null when the team number is not configured yet
   */
  public static String staticIpForTeam(int teamNumber) {
    if (teamNumber <= 0) {
      return null;
    }
    return String.format("10.%d.%d.11", teamNumber / 100, teamNumber % 100);
  }

  private static void forwardRange(int localBasePort, String host) {
    for (int i = 0; i < 10; i++) {
      PortForwarder.add(localBasePort + i, host, 5800 + i);
    }
  }

  /**
   * @deprecated Use {@link #setupPortForwarding()}, which sets up this route alongside the others
   *     instead of fighting them for local ports.
   */
  @Deprecated
  public static void setupPortForwardingUSB(int usbIndex) {
        String ip = "172.29." + usbIndex + ".1";
        int basePort = PORT_BASE_USB + (usbIndex * 10);

        for (int i = 0; i < 10; i++) {
            PortForwarder.add(basePort + i, ip, 5800 + i);
        }
    }

  public static void setupPortForwardingRobotWifi(String host) {
        setupPortForwarding(host, 5800);
    }

  public static void setupPortForwardingRobotWifi() {
        setupPortForwarding("limelight.local", 5800);
    }

  private static void setupPortForwarding(String host, int basePort) {
        for (int i = 0; i < 10; i++) {
            PortForwarder.add(basePort + i, host, 5800 + i);
        }
    }

  /**
   * Spins the robot in place until robot center is aimed at the target.
   * Uses getCorrectedTX() to account for limelight being offset from center.
   * Bind this to a button in RobotContainer.
   */
  public Command alignCommand(SwerveSubsystem drivebase) {
    return Commands.run(
            () -> drivebase.drive(
                Translation2d.kZero,   // no translation — spin in place
                getAlignOmega(),       // proportional rotation toward corrected target
                true                   // field-relative keeps heading stable
            ),
            drivebase
        )
        .until(this::isCentered)
        .withName("LimeLight.align");
        //this should work now :)
  }

  public Command alignToTagsCommand(SwerveSubsystem drivebase, Set<Integer> allowedTagIds) {
    return Commands.runEnd(
            () -> {
              AprilTagScan scan = scan(allowedTagIds);
              if (!scan.isValid()) {
                filteredClimbTxDegrees = 0.0;
                filteredClimbArea = 0.0;
                lastRawClimbTxDegrees = 0.0;
                lastClimbForward = 0.0;
                hasClimbAlignSample = false;
                climbJumpHoldCyclesRemaining = 0;
                drivebase.drive(Translation2d.kZero, 0.0, false);
                return;
              }

              double robotOmegaDegPerSecond = Math.toDegrees(drivebase.getRobotVelocity().omegaRadiansPerSecond);
              double visionLatencySeconds = (scan.latencyMs + CLIMB_ALIGN_LATENCY_FUDGE_MS) / 1000.0;
              double latencyCompensationDegrees = robotOmegaDegPerSecond * visionLatencySeconds;
              double rawTxDegrees = scan.tx + latencyCompensationDegrees;
              double rawArea = getTA();

              if (!hasClimbAlignSample) {
                filteredClimbTxDegrees = rawTxDegrees;
                filteredClimbArea = rawArea;
                lastRawClimbTxDegrees = rawTxDegrees;
                hasClimbAlignSample = true;
              }

              double txDeltaDegrees = rawTxDegrees - lastRawClimbTxDegrees;
              if (Math.abs(txDeltaDegrees) > CLIMB_ALIGN_MAX_TX_JUMP_DEGREES) {
                climbJumpHoldCyclesRemaining = CLIMB_ALIGN_JUMP_HOLD_CYCLES;
              } else if (climbJumpHoldCyclesRemaining > 0) {
                climbJumpHoldCyclesRemaining--;
              }
              lastRawClimbTxDegrees = rawTxDegrees;

              if (climbJumpHoldCyclesRemaining == 0) {
                filteredClimbTxDegrees +=
                    (rawTxDegrees - filteredClimbTxDegrees) * CLIMB_ALIGN_TX_FILTER_ALPHA;
                filteredClimbArea +=
                    (rawArea - filteredClimbArea) * CLIMB_ALIGN_AREA_FILTER_ALPHA;
              }

              double filteredDistanceMeters =
                  Math.max(calculateDistanceFromArea(Math.max(filteredClimbArea, 1e-6)), 0.0);
              AprilTagScan filteredScan = new AprilTagScan(
                  true,
                  scan.tagID,
                  filteredClimbTxDegrees,
                  scan.ty,
                  filteredDistanceMeters,
                  scan.latencyMs,
                  scan.pose);
              double correctedTx = getCorrectedTX(filteredScan);
              double omega = MathUtil.clamp(
                  -correctedTx * ALIGN_KP,
                  -ALIGN_MAX_OMEGA_RAD_S,
                  ALIGN_MAX_OMEGA_RAD_S);
              double areaError = CLIMB_ALIGN_DESIRED_AREA - filteredClimbArea;
              double requestedForward = MathUtil.clamp(
                  areaError * CLIMB_ALIGN_FORWARD_KP,
                  -CLIMB_ALIGN_MAX_FORWARD,
                  CLIMB_ALIGN_MAX_FORWARD);
              double forwardMagnitudeCap =
                  Math.abs(areaError) < CLIMB_ALIGN_SLOW_AREA_ERROR
                      ? CLIMB_ALIGN_CLOSE_MAX_FORWARD
                      : CLIMB_ALIGN_MAX_FORWARD;
              if (Math.abs(correctedTx) > CLIMB_ALIGN_TURN_ONLY_ERROR_DEGREES) {
                forwardMagnitudeCap = 0.0;
              }
              requestedForward = MathUtil.clamp(
                  requestedForward,
                  -forwardMagnitudeCap,
                  forwardMagnitudeCap);
              double forward = MathUtil.clamp(
                  requestedForward,
                  lastClimbForward - CLIMB_ALIGN_MAX_FORWARD_STEP,
                  lastClimbForward + CLIMB_ALIGN_MAX_FORWARD_STEP);
              lastClimbForward = forward;

              drivebase.drive(
                  new Translation2d(forward, 0.0),
                  omega,
                  false);
            },
            () -> {
              filteredClimbTxDegrees = 0.0;
              filteredClimbArea = 0.0;
              lastRawClimbTxDegrees = 0.0;
              lastClimbForward = 0.0;
              hasClimbAlignSample = false;
              climbJumpHoldCyclesRemaining = 0;
              drivebase.drive(Translation2d.kZero, 0.0, false);
            },
            drivebase)
        .until(() -> {
          AprilTagScan scan = scan(allowedTagIds);
          return scan.isValid()
              && hasClimbAlignSample
              && Math.abs(getCorrectedTX(new AprilTagScan(
                  true,
                  scan.tagID,
                  filteredClimbTxDegrees,
                  scan.ty,
                  Math.max(calculateDistanceFromArea(Math.max(filteredClimbArea, 1e-6)), 0.0),
                  scan.latencyMs,
                  scan.pose))) < ALIGN_TOLERANCE_DEGREES
              && Math.abs(CLIMB_ALIGN_DESIRED_AREA - filteredClimbArea) < CLIMB_ALIGN_AREA_TOLERANCE;
        })
        .withName("LimeLight.alignToTags");
  }

  public static class AprilTagScan {
    public final boolean hasTarget;
    public final int tagID;
    public final double tx;
    public final double ty;
    public final double distance;
    public final double latencyMs;
    public final Pose3d pose;

    public AprilTagScan(boolean hasTarget, int tagID, double tx, double ty, double distance,
        double latencyMs, Pose3d pose) {
        this.hasTarget = hasTarget;
        this.tagID = tagID;
        this.tx = tx;
        this.ty = ty;
        this.distance = distance;
        this.latencyMs = latencyMs;
        this.pose = pose;
    }

    public boolean isValid() {
        return hasTarget && tagID != -1;
    }
}

public AprilTagScan scan() {
    refreshScanCacheIfNeeded();
    AprilTagScan closestScan = scanClosestFiducialFromCache(null);
    if (closestScan != null) {
      return closestScan;
    }

    return cachedAnyScan;
}

public AprilTagScan scanContinuous() {
    refreshScanCacheIfNeeded();
    return cachedAnyScan;
}

public AprilTagScan scan(Set<Integer> allowedTagIds) {
    refreshScanCacheIfNeeded();
    AprilTagScan closestScan = scanClosestFiducialFromCache(allowedTagIds);
    if (closestScan != null) {
        return closestScan;
    }

    if (!cachedAnyScan.isValid() || allowedTagIds.contains(cachedAnyScan.tagID)) {
        return cachedAnyScan;
    }

    return new AprilTagScan(
        false,
        -1,
        cachedAnyScan.tx,
        cachedAnyScan.ty,
        0.0,
        cachedAnyScan.latencyMs,
        new Pose3d()
    );
}

public AprilTagScan scanDirect() {
    return scanDirect(null);
}

public AprilTagScan scanDirect(Set<Integer> allowedTagIds) {
    double tv = tvEntry.getDouble(0);
    double tx = txEntry.getDouble(0);
    double ty = tyEntry.getDouble(0);
    double ta = taEntry.getDouble(0);
    int tagId = (int) tidEntry.getDouble(-1);
    double latencyMs = tlEntry.getDouble(0.0) + clEntry.getDouble(0.0);
    double distance = tv == 1 ? calculateDistanceFromArea(ta) : 0.0;
    if (!Double.isFinite(distance)
        || distance < MIN_REASONABLE_TAG_DISTANCE_METERS
        || distance > MAX_REASONABLE_TAG_DISTANCE_METERS) {
      distance = 0.0;
    }
    Pose3d pose = (tv == 1 && distance > 0.0)
        ? calculatePoseFromAngles(tx, ty, distance)
        : new Pose3d();
    AprilTagScan anyScan = new AprilTagScan(
        tv == 1,
        tagId,
        tx,
        ty,
        distance,
        latencyMs,
        pose);

    double[] rawFiducials = rawFiducialsEntry.getDoubleArray(new double[0]);
    AprilTagScan closestScan = scanClosestFiducialFromRaw(rawFiducials, allowedTagIds, latencyMs);
    AprilTagScan result;
    if (closestScan != null) {
      result = closestScan;
    } else if (!anyScan.isValid() || allowedTagIds == null || allowedTagIds.contains(anyScan.tagID)) {
      result = anyScan;
    } else {
      result = new AprilTagScan(
          false,
          -1,
          anyScan.tx,
          anyScan.ty,
          0.0,
          anyScan.latencyMs,
          new Pose3d());
    }

    Logger.recordOutput("Limelight/Direct/HasTarget", result.hasTarget);
    Logger.recordOutput("Limelight/Direct/TagID", result.tagID);
    Logger.recordOutput("Limelight/Direct/TX", result.tx);
    Logger.recordOutput("Limelight/Direct/TY", result.ty);
    Logger.recordOutput("Limelight/Direct/DistanceM", result.distance);
    Logger.recordOutput("Limelight/Direct/LatencyMs", result.latencyMs);
    Logger.recordOutput("Limelight/Direct/RawFiducialArrayLength", rawFiducials.length);
    return result;
}

private AprilTagScan scanClosestFiducialFromCache(Set<Integer> allowedTagIds) {
    return scanClosestFiducialFromRaw(cachedRawFiducials, allowedTagIds, getTotalLatencyMs());
}

private AprilTagScan scanClosestFiducialFromRaw(
    double[] raw,
    Set<Integer> allowedTagIds,
    double latencyMs) {
    if (raw.length < RAW_FIDUCIAL_STRIDE) {
        return null;
    }

    boolean found = false;
    int bestTagId = -1;
    double bestTx = 0.0;
    double bestTy = 0.0;
    double bestDistanceMeters = 0.0;
    double bestDistanceScore = Double.POSITIVE_INFINITY;

    for (int i = 0; i + RAW_FIDUCIAL_STRIDE - 1 < raw.length; i += RAW_FIDUCIAL_STRIDE) {
        int tagId = (int) raw[i + RAW_FIDUCIAL_ID_INDEX];
        if (allowedTagIds != null && !allowedTagIds.contains(tagId)) {
            continue;
        }

        double tx = raw[i + RAW_FIDUCIAL_TX_INDEX];
        double ty = raw[i + RAW_FIDUCIAL_TY_INDEX];
        double ta = raw[i + RAW_FIDUCIAL_TA_INDEX];
        double distanceMeters = raw[i + RAW_FIDUCIAL_DISTANCE_INDEX];

        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0.0) {
            distanceMeters = calculateDistanceFromArea(ta);
        }
        if (!Double.isFinite(distanceMeters)
            || distanceMeters < MIN_REASONABLE_TAG_DISTANCE_METERS
            || distanceMeters > MAX_REASONABLE_TAG_DISTANCE_METERS) {
            distanceMeters = calculateDistanceFromArea(ta);
        }
        if (!Double.isFinite(distanceMeters)
            || distanceMeters <= 0.0
            || distanceMeters < MIN_REASONABLE_TAG_DISTANCE_METERS
            || distanceMeters > MAX_REASONABLE_TAG_DISTANCE_METERS) {
            continue;
        }

        if (!found || distanceMeters < bestDistanceScore) {
            found = true;
            bestDistanceScore = distanceMeters;
            bestTagId = tagId;
            bestTx = tx;
            bestTy = ty;
            bestDistanceMeters = distanceMeters;
        }
    }

    if (!found) {
        return null;
    }

    Pose3d pose = calculatePoseFromAngles(bestTx, bestTy, bestDistanceMeters);
    return new AprilTagScan(
        true,
        bestTagId,
        bestTx,
        bestTy,
        bestDistanceMeters,
        latencyMs,
        pose);
}

private void refreshScanCacheIfNeeded() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastScanCacheTimestampSeconds >= 0.0
        && nowSeconds - lastScanCacheTimestampSeconds < SCAN_CACHE_PERIOD_SECONDS) {
      return;
    }

    cachedTv = tvEntry.getDouble(0);
    double tx = txEntry.getDouble(0);
    double ty = tyEntry.getDouble(0);
    cachedTa = taEntry.getDouble(0);
    int tagId = (int) tidEntry.getDouble(-1);
    double latencyMs = tlEntry.getDouble(0.0) + clEntry.getDouble(0.0);
    double distance = cachedTv == 1 ? calculateDistanceFromArea(cachedTa) : 0.0;
    if (!Double.isFinite(distance)
        || distance < MIN_REASONABLE_TAG_DISTANCE_METERS
        || distance > MAX_REASONABLE_TAG_DISTANCE_METERS) {
      distance = 0.0;
    }
    Pose3d pose = (cachedTv == 1 && distance > 0.0)
        ? calculatePoseFromAngles(tx, ty, distance)
        : new Pose3d();
    cachedAnyScan = new AprilTagScan(
        cachedTv == 1,
        tagId,
        tx,
        ty,
        distance,
        latencyMs,
        pose);
    cachedRawFiducials = rawFiducialsEntry.getDoubleArray(new double[0]);
    lastScanCacheTimestampSeconds = nowSeconds;

    // The camera bumps tv's change counter every frame it publishes. If that counter stops moving,
    // the Limelight is not talking to us at all — a different problem from "sees no tags", and one
    // no amount of aiming the camera will fix.
    long ntChange = tvEntry.getLastChange();
    if (ntChange != lastNtChangeMicros) {
      lastNtChangeMicros = ntChange;
      lastCameraUpdateSeconds = nowSeconds;
    }
    reportConnectionIfDown(nowSeconds);
    reportAddressOnceConnected(nowSeconds);
    Logger.recordOutput("Limelight/Connected", isConnected());
    Logger.recordOutput("Limelight/SecondsSinceUpdate", getSecondsSinceLastUpdate());

    Logger.recordOutput("Limelight/Cached/HasTarget", cachedAnyScan.hasTarget);
    Logger.recordOutput("Limelight/Cached/TagID", cachedAnyScan.tagID);
    Logger.recordOutput("Limelight/Cached/TX", cachedAnyScan.tx);
    Logger.recordOutput("Limelight/Cached/TY", cachedAnyScan.ty);
    Logger.recordOutput("Limelight/Cached/TA", cachedTa);
    Logger.recordOutput("Limelight/Cached/DistanceM", cachedAnyScan.distance);
    Logger.recordOutput("Limelight/Cached/LatencyMs", cachedAnyScan.latencyMs);
    Logger.recordOutput("Limelight/Cached/RawFiducialArrayLength", cachedRawFiducials.length);
    Logger.recordOutput("Limelight/Cached/TvTidConsistent", !(cachedAnyScan.hasTarget && cachedAnyScan.tagID == -1));
    logRawFiducialDiagnostics();
}

private void logRawFiducialDiagnostics() {
    int rawArrayLength = cachedRawFiducials.length;
    int candidateCount = 0;
    int finiteDistanceCount = 0;
    int fallbackDistanceCount = 0;
    int usableCount = 0;
    int rejectedByDistanceCount = 0;
    int rejectedByInvalidCount = 0;
    int bestTagId = -1;
    double bestDistanceMeters = Double.POSITIVE_INFINITY;
    double bestTx = 0.0;
    double bestTy = 0.0;

    for (int i = 0; i + RAW_FIDUCIAL_STRIDE - 1 < rawArrayLength; i += RAW_FIDUCIAL_STRIDE) {
      candidateCount++;
      int tagId = (int) cachedRawFiducials[i + RAW_FIDUCIAL_ID_INDEX];
      double tx = cachedRawFiducials[i + RAW_FIDUCIAL_TX_INDEX];
      double ty = cachedRawFiducials[i + RAW_FIDUCIAL_TY_INDEX];
      double ta = cachedRawFiducials[i + RAW_FIDUCIAL_TA_INDEX];
      double distanceMeters = cachedRawFiducials[i + RAW_FIDUCIAL_DISTANCE_INDEX];

      if (Double.isFinite(distanceMeters) && distanceMeters > 0.0) {
        finiteDistanceCount++;
      } else {
        distanceMeters = calculateDistanceFromArea(ta);
        fallbackDistanceCount++;
      }

      if (!Double.isFinite(distanceMeters) || distanceMeters <= 0.0) {
        rejectedByInvalidCount++;
        continue;
      }

      if (distanceMeters < MIN_REASONABLE_TAG_DISTANCE_METERS
          || distanceMeters > MAX_REASONABLE_TAG_DISTANCE_METERS) {
        rejectedByDistanceCount++;
        continue;
      }

      usableCount++;
      if (distanceMeters < bestDistanceMeters) {
        bestDistanceMeters = distanceMeters;
        bestTagId = tagId;
        bestTx = tx;
        bestTy = ty;
      }
    }

    Logger.recordOutput("Limelight/RawFiducials/CandidateCount", candidateCount);
    Logger.recordOutput("Limelight/RawFiducials/FiniteDistanceCount", finiteDistanceCount);
    Logger.recordOutput("Limelight/RawFiducials/FallbackDistanceCount", fallbackDistanceCount);
    Logger.recordOutput("Limelight/RawFiducials/UsableCount", usableCount);
    Logger.recordOutput("Limelight/RawFiducials/RejectedByDistanceCount", rejectedByDistanceCount);
    Logger.recordOutput("Limelight/RawFiducials/RejectedByInvalidCount", rejectedByInvalidCount);
    Logger.recordOutput("Limelight/RawFiducials/ClosestTagId", bestTagId);
    Logger.recordOutput(
        "Limelight/RawFiducials/ClosestDistanceM",
        bestTagId == -1 ? 0.0 : bestDistanceMeters);
    Logger.recordOutput("Limelight/RawFiducials/ClosestTx", bestTx);
    Logger.recordOutput("Limelight/RawFiducials/ClosestTy", bestTy);
}

}
