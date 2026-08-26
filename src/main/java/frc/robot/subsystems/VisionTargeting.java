package frc.robot.subsystems;

import frc.robot.FieldConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

/**
 * Converts a raw Limelight 3A observation into robot-frame geometry.
 *
 * <p>This is the only place in the project that touches Limelight sign conventions or mounting
 * offsets. The turret, the shooter, and the drivebase lineup all consume the same
 * {@link TargetObservation}, so they can never disagree about where the tag is -- which is exactly
 * what used to happen when each subsystem re-derived distance and bearing its own way.
 *
 * <h2>Why absolute geometry instead of "turret angle plus tx"</h2>
 *
 * <p>The camera is chassis-fixed, so tx is the angle from the LENS to the tag, not turret error.
 * The old approach commanded {@code turretAngle = currentTurretAngle + f(tx)}, which puts the
 * turret's own measured position inside its own command -- any encoder noise or lag feeds straight
 * back into the setpoint and the loop hunts. Here the tag is resolved to a fixed point in the robot
 * frame and the turret angle that points at that point is computed outright. The command depends
 * only on the camera measurement, so there is no feedback path through the turret.
 *
 * <h2>Coordinate conventions</h2>
 *
 * <ul>
 *   <li>Robot frame: +X forward, +Y left, +Z up, angles CCW-positive (WPILib standard).
 *   <li>Limelight: +tx means the target is to the RIGHT, +ty means the target is UP.
 * </ul>
 *
 * <p>The tx sign flip happens once, in {@link #solve}. Nothing downstream should negate tx again.
 */
public final class VisionTargeting {

  private VisionTargeting() {
  }

  /** Result of resolving one Limelight frame into robot-frame geometry. */
  public static final class TargetObservation {

    /** Sentinel used whenever there is nothing usable to aim at. */
    public static final TargetObservation INVALID =
        new TargetObservation(false, false, -1, 0, 0, 0, 0, 0, 0, 0, "NONE", "no target");

    /** True when every field below is trustworthy. */
    public final boolean valid;

    /**
     * True when the camera did see an allowed tag. Combined with {@link #valid} being false, this
     * separates "nothing in view" from "saw a tag but could not turn it into geometry" — two very
     * different problems that deserve different messages on the driver station.
     */
    public final boolean hadRawTarget;

    /** Fiducial ID this observation came from, or -1. */
    public final int tagId;

    /** CCW-positive angle from robot forward to the tag, measured at robot center, in degrees. */
    public final double robotBearingDegrees;

    /** Horizontal ground range from robot center to the tag, in meters. */
    public final double robotDistanceMeters;

    /**
     * Absolute turret encoder angle, in degrees, that aims the shooter at the tag. Already includes
     * {@link TurretConstants#ENCODER_OFFSET_DEGREES}, so it can be commanded directly.
     */
    public final double turretAngleDegrees;

    /** Horizontal ground range from the ball exit point to the tag, in meters. */
    public final double shooterDistanceMeters;

    /** Tag position in the robot frame, in meters. Useful for logging and for field overlays. */
    public final double targetXMeters;
    public final double targetYMeters;

    /** Total measurement age at the moment this was solved, in seconds. */
    public final double latencySeconds;

    /** Which range method produced the distance: TY_TRIG or LL3D_FALLBACK. */
    public final String distanceSource;

    /** Human-readable reason, mainly to explain an invalid observation. */
    public final String message;

    private TargetObservation(
        boolean valid,
        boolean hadRawTarget,
        int tagId,
        double robotBearingDegrees,
        double robotDistanceMeters,
        double turretAngleDegrees,
        double shooterDistanceMeters,
        double targetXMeters,
        double targetYMeters,
        double latencySeconds,
        String distanceSource,
        String message) {
      this.valid = valid;
      this.hadRawTarget = hadRawTarget;
      this.tagId = tagId;
      this.robotBearingDegrees = robotBearingDegrees;
      this.robotDistanceMeters = robotDistanceMeters;
      this.turretAngleDegrees = turretAngleDegrees;
      this.shooterDistanceMeters = shooterDistanceMeters;
      this.targetXMeters = targetXMeters;
      this.targetYMeters = targetYMeters;
      this.latencySeconds = latencySeconds;
      this.distanceSource = distanceSource;
      this.message = message;
    }

    /** A tag was visible but its geometry could not be resolved. */
    static TargetObservation invalid(String reason) {
      return new TargetObservation(false, true, -1, 0, 0, 0, 0, 0, 0, 0, "NONE", reason);
    }

    /** True when the turret can physically reach the angle this observation asks for. */
    public boolean isWithinTurretRange() {
      return valid && Math.abs(turretAngleDegrees) <= TurretConstants.MAX_ONE_DIR_FOV_DEGREES;
    }

    /** True when the shooter range is inside the window the shot model is trusted over. */
    public boolean isWithinShootingRange() {
      return valid
          && shooterDistanceMeters >= ShooterConstants.MIN_VALID_DISTANCE_METERS
          && shooterDistanceMeters <= ShooterConstants.MAX_VALID_DISTANCE_METERS;
    }

    @Override
    public String toString() {
      if (!valid) {
        return "TargetObservation[INVALID: " + message + "]";
      }
      return String.format(
          "TargetObservation[tag=%d | bearing=%.2f deg | range=%.2f m | turret=%.2f deg | shot=%.2f m | %s]",
          tagId,
          robotBearingDegrees,
          robotDistanceMeters,
          turretAngleDegrees,
          shooterDistanceMeters,
          distanceSource);
    }
  }

  /**
   * Resolves a Limelight scan into robot-frame geometry, with no motion compensation.
   *
   * @param scan the closest matching fiducial from {@link LimeLight#scan}
   */
  public static TargetObservation solve(LimeLight.AprilTagScan scan) {
    return solve(scan, 0.0);
  }

  /**
   * Resolves a Limelight scan into robot-frame geometry.
   *
   * <p>Vision is old by the time it arrives. If the chassis has been yawing, the tag has swept
   * across the robot frame since the shutter fired, so the solved point is rotated forward by the
   * yaw accumulated over the measurement age. Translation during that window is ignored: at typical
   * latency it moves the answer far less than the tag-centering noise does.
   *
   * @param scan                 the closest matching fiducial
   * @param robotOmegaRadPerSec  current chassis yaw rate, CCW-positive, used for latency
   *                             compensation. Pass 0 to skip compensation.
   */
  public static TargetObservation solve(LimeLight.AprilTagScan scan, double robotOmegaRadPerSec) {
    if (scan == null || !scan.isValid()) {
      return TargetObservation.INVALID;
    }
    if (!Double.isFinite(scan.tx) || !Double.isFinite(scan.ty)) {
      return TargetObservation.invalid("non-finite tx/ty from Limelight");
    }

    // 1. Build the unit ray to the tag in the camera frame. Camera frame matches the robot
    //    convention (+X out the lens, +Y left, +Z up), so the right-positive tx is negated here --
    //    the one and only place that flip happens.
    double vx = 1.0;
    double vy = -Math.tan(Math.toRadians(scan.tx));
    double vz = Math.tan(Math.toRadians(scan.ty));

    // 2. Rotate the ray out of the camera frame into the robot frame: pitch up, then yaw CCW.
    //    Done exactly rather than with the usual small-angle "distance = dh / tan(pitch + ty)"
    //    shortcut, which drifts once the camera is pitched steeply and the tag is off to one side.
    double pitch = Math.toRadians(LimeLightConstants.MOUNT_PITCH_DEGREES);
    double pitchedX = vx * Math.cos(pitch) - vz * Math.sin(pitch);
    double pitchedZ = vx * Math.sin(pitch) + vz * Math.cos(pitch);
    double pitchedY = vy;

    double yaw = Math.toRadians(LimeLightConstants.MOUNT_YAW_DEGREES);
    double rayX = pitchedX * Math.cos(yaw) - pitchedY * Math.sin(yaw);
    double rayY = pitchedX * Math.sin(yaw) + pitchedY * Math.cos(yaw);
    double rayZ = pitchedZ;

    // 3. Walk the ray out until it reaches the tag's known height. This is the range measurement:
    //    it depends only on tx, ty and fixed geometry, so it is far steadier than the on-board 3D
    //    pose solve, which gets ambiguous on a single tag at range.
    //
    //    The height is looked up per tag ID, because the 2026 field mounts HUB tags at 44.25 in,
    //    TRENCH at 35 in and TOWER/OUTPOST at 21.75 in. Assuming one height for all of them skews
    //    range by roughly 25 percent the moment a different structure comes into view.
    double deltaHeight =
        FieldConstants.tagHeightMeters(scan.tagID) - LimeLightConstants.MOUNT_HEIGHT_METERS;
    double horizontalFromCamera;
    String distanceSource;

    double rayScale = Math.abs(rayZ) > 1e-6 ? deltaHeight / rayZ : Double.NaN;
    if (Double.isFinite(rayScale) && rayScale > 0.0) {
      horizontalFromCamera = rayScale * Math.hypot(rayX, rayY);
      distanceSource = "TY_TRIG";
    } else {
      // The ray never reaches the tag plane, which means ty is at or past the horizon for this
      // mounting. Fall back to the range LimeLight already resolved, converting slant range to
      // ground range. LimeLight.scanClosestFiducialFromRaw has already rejected anything outside
      // MIN/MAX_REASONABLE_TAG_DISTANCE_METERS, so this is gated even without an ambiguity term.
      double slantRange = scan.distance;
      double groundSquared = (slantRange * slantRange) - (deltaHeight * deltaHeight);
      if (slantRange <= 0.0 || groundSquared <= 0.0) {
        return TargetObservation.invalid("no usable range from ty trig or 3D solve");
      }
      horizontalFromCamera = Math.sqrt(groundSquared);
      distanceSource = "LL3D_FALLBACK";
    }

    if (!Double.isFinite(horizontalFromCamera) || horizontalFromCamera <= 0.0) {
      return TargetObservation.invalid("computed non-positive range");
    }

    // 4. Place the tag in the robot frame, starting from where the lens actually is.
    double bearingFromCamera = Math.atan2(rayY, rayX);
    double targetX =
        LimeLightConstants.MOUNT_FORWARD_METERS + horizontalFromCamera * Math.cos(bearingFromCamera);
    double targetY =
        LimeLightConstants.MOUNT_LEFT_METERS + horizontalFromCamera * Math.sin(bearingFromCamera);

    // 5. Age the measurement forward. The chassis rotated CCW by omega*latency since the shutter
    //    fired, so a stationary tag has swung the opposite way in the robot frame.
    double latencySeconds =
        (scan.latencyMs + LimeLightConstants.LATENCY_FUDGE_MS) / 1000.0;
    if (robotOmegaRadPerSec != 0.0 && Double.isFinite(robotOmegaRadPerSec)) {
      double unwind = -robotOmegaRadPerSec * latencySeconds;
      double cos = Math.cos(unwind);
      double sin = Math.sin(unwind);
      double rotatedX = targetX * cos - targetY * sin;
      double rotatedY = targetX * sin + targetY * cos;
      targetX = rotatedX;
      targetY = rotatedY;
    }

    // 6. Derive what each consumer actually needs.
    double robotBearingDegrees = Math.toDegrees(Math.atan2(targetY, targetX));
    double robotDistanceMeters = Math.hypot(targetX, targetY);

    double pivotX = TurretConstants.PIVOT_TO_ROBOT_CENTER.getX();
    double pivotY = TurretConstants.PIVOT_TO_ROBOT_CENTER.getY();
    double pivotToTargetX = targetX - pivotX;
    double pivotToTargetY = targetY - pivotY;

    double turretAngleDegrees =
        Math.toDegrees(Math.atan2(pivotToTargetY, pivotToTargetX))
            + TurretConstants.ENCODER_OFFSET_DEGREES;
    double pivotDistanceMeters = Math.hypot(pivotToTargetX, pivotToTargetY);

    // Once aimed, the ball leaves the shooter radius out along the turret axis, so the shot travels
    // that much less than the pivot-to-tag range.
    double shooterDistanceMeters =
        Math.max(0.0, pivotDistanceMeters - TurretConstants.SHOOTER_EXIT_RADIUS_METERS);

    return new TargetObservation(
        true,
        true,
        scan.tagID,
        robotBearingDegrees,
        robotDistanceMeters,
        turretAngleDegrees,
        shooterDistanceMeters,
        targetX,
        targetY,
        latencySeconds,
        distanceSource,
        "ok");
  }

  /**
   * Convenience wrapper: scan for a hub tag and solve it in one call.
   *
   * @param limelight            camera to read
   * @param robotOmegaRadPerSec  chassis yaw rate for latency compensation, or 0
   */
  public static TargetObservation observeHub(LimeLight limelight, double robotOmegaRadPerSec) {
    return solve(limelight.scan(FieldConstants.hubTagIds()), robotOmegaRadPerSec);
  }
}
