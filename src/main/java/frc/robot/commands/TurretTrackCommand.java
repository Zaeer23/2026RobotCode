package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionTargeting.TargetObservation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Holds the turret on a HUB AprilTag using closed-loop position control.
 *
 * <h2>Why this replaces the open-loop tx chase</h2>
 *
 * <p>The previous tracking step drove duty cycle proportional to tx, with a minimum output floor.
 * Two problems came out of that:
 *
 * <ul>
 *   <li><b>The setpoint depended on the turret's own position.</b> Aim was expressed as "current
 *       angle minus filtered tx", so encoder lag and vision lag both fed back into the command and
 *       the loop chased itself.
 *   <li><b>The output floor guaranteed hunting.</b> Any error past the deadband jumped straight to
 *       TURRET_TRACK_MIN_OUTPUT, which overshot back through the deadband, so the turret oscillated
 *       around the target instead of settling on it.
 * </ul>
 *
 * <p>Here {@link frc.robot.subsystems.VisionTargeting} resolves the tag to a fixed point in the
 * robot frame and reports the ABSOLUTE turret angle that aims at it. That angle is a pure function
 * of the camera measurement, so there is no feedback path through the turret at all, and it is
 * handed to {@link TurretSubsystem#setClosedLoopAngle} so the existing position controller does the
 * work it was tuned for.
 */
public class TurretTrackCommand extends Command {

  /** Smoothing on the solved aim angle. Higher tracks faster and shakes more. */
  private static final double ANGLE_FILTER_ALPHA = 0.35;

  /** Maximum change in commanded angle per 20 ms loop, so fast tag motion cannot demand a slam. */
  private static final double MAX_TARGET_STEP_DEGREES = 2.0;

  /** A one-frame jump larger than this is a glitch, not real motion. */
  private static final double MAX_ANGLE_JUMP_DEGREES = 15.0;

  private static final int JUMP_HOLD_CYCLES = 5;

  /** Errors below this count as zero, so the turret stops fidgeting once it is there. */
  private static final double DEADBAND_DEGREES = 0.25;

  /** How often the console/driver-station gets a tracking line, in seconds. */
  private static final double LOG_PERIOD_SECONDS = 0.5;

  private final TurretSubsystem turret;
  private final LimeLight limelight;
  private final SwerveSubsystem drivebase;
  private final Supplier<Set<Integer>> allowedTagIds;

  private double filteredTargetAngleDegrees = 0.0;
  private double lastRawTargetAngleDegrees = 0.0;
  private double lastCommandedAngleDegrees = 0.0;
  private int lastTagId = -1;
  private boolean hasSample = false;
  private int jumpHoldCyclesRemaining = 0;
  private double lastLogTimestampSeconds = -1.0;

  public TurretTrackCommand(
      TurretSubsystem turret, LimeLight limelight, SwerveSubsystem drivebase) {
    this(turret, limelight, drivebase, FieldConstants::hubTagIds);
  }

  /**
   * @param allowedTagIds supplier rather than a fixed set, because commands are constructed at boot
   *                      but the driver station does not report an alliance until it connects. A
   *                      set captured at construction time would pin the robot to whichever
   *                      alliance was assumed at power-on.
   */
  public TurretTrackCommand(
      TurretSubsystem turret,
      LimeLight limelight,
      SwerveSubsystem drivebase,
      Supplier<Set<Integer>> allowedTagIds) {
    this.turret = turret;
    this.limelight = limelight;
    this.drivebase = drivebase;
    this.allowedTagIds = allowedTagIds;
    addRequirements(turret);
    setName("Turret.track");
  }

  @Override
  public void initialize() {
    hasSample = false;
    lastTagId = -1;
    jumpHoldCyclesRemaining = 0;
    lastLogTimestampSeconds = -1.0;
    lastCommandedAngleDegrees = turret.getRawAngle().in(Degrees);
    turret.clearTrackingTelemetry();

    // Deliberately does NOT push a camera-side tag filter. limelight.observe() already picks the
    // closest allowed tag in software, which is stateless. The camera-side filter did the same job
    // but persisted on the hardware if this command never reached end(), leaving the Limelight
    // apparently blind to every other tag until someone power-cycled it.
    limelight.clearTagFilter();
  }

  @Override
  public void execute() {
    TargetObservation observation = limelight.observe(allowedTagIds.get(), drivebase);
    double currentAngleDegrees = turret.getRawAngle().in(Degrees);

    if (!observation.valid) {
      hasSample = false;
      jumpHoldCyclesRemaining = 0;
      turret.clearTrackingTelemetry();
      turret.setTrackingError(Double.POSITIVE_INFINITY);
      // Hold station rather than snapping anywhere. A dropout is usually a single frame.
      turret.setClosedLoopAngle(Degrees.of(lastCommandedAngleDegrees));
      maybeLog("NO_TARGET", observation, currentAngleDegrees, lastCommandedAngleDegrees);
      reportWhyNoTarget();
      return;
    }

    double rawTargetAngleDegrees = observation.turretAngleDegrees;

    if (observation.tagId != lastTagId) {
      // Switching tags moves the aim point discontinuously; re-seed rather than filter across it.
      hasSample = false;
      jumpHoldCyclesRemaining = 0;
      lastTagId = observation.tagId;
    }

    if (!hasSample) {
      filteredTargetAngleDegrees = rawTargetAngleDegrees;
      lastRawTargetAngleDegrees = rawTargetAngleDegrees;
      lastCommandedAngleDegrees = currentAngleDegrees;
      hasSample = true;
    }

    // Freeze briefly on an unrealistic one-frame jump instead of commanding a violent reversal.
    double angleDeltaDegrees = rawTargetAngleDegrees - lastRawTargetAngleDegrees;
    if (Math.abs(angleDeltaDegrees) > MAX_ANGLE_JUMP_DEGREES) {
      jumpHoldCyclesRemaining = JUMP_HOLD_CYCLES;
    } else if (jumpHoldCyclesRemaining > 0) {
      jumpHoldCyclesRemaining--;
    }
    lastRawTargetAngleDegrees = rawTargetAngleDegrees;

    if (jumpHoldCyclesRemaining == 0) {
      filteredTargetAngleDegrees +=
          (rawTargetAngleDegrees - filteredTargetAngleDegrees) * ANGLE_FILTER_ALPHA;
    }

    double steppedTargetDegrees = MathUtil.clamp(
        filteredTargetAngleDegrees,
        lastCommandedAngleDegrees - MAX_TARGET_STEP_DEGREES,
        lastCommandedAngleDegrees + MAX_TARGET_STEP_DEGREES);
    double commandedDegrees = MathUtil.clamp(
        steppedTargetDegrees,
        -TurretConstants.MAX_ONE_DIR_FOV_DEGREES,
        TurretConstants.MAX_ONE_DIR_FOV_DEGREES);
    lastCommandedAngleDegrees = commandedDegrees;

    turret.setClosedLoopAngle(Degrees.of(commandedDegrees));

    // Report error against the UNFILTERED solution, so "on target" means actually on the tag and
    // not merely settled onto a stale filtered value.
    double errorDegrees = rawTargetAngleDegrees - currentAngleDegrees;
    if (Math.abs(errorDegrees) < DEADBAND_DEGREES) {
      errorDegrees = 0.0;
    }
    turret.setTrackingError(errorDegrees);
    turret.updateTrackingTelemetry(observation.tagId, errorDegrees, 0.0);

    Logger.recordOutput("Turret/RawTargetAngleDegrees", rawTargetAngleDegrees);
    Logger.recordOutput("Turret/FilteredTargetAngleDegrees", filteredTargetAngleDegrees);
    Logger.recordOutput("Turret/TrackingErrorDegrees", errorDegrees);
    Logger.recordOutput("Turret/JumpHoldCycles", jumpHoldCyclesRemaining);
    Logger.recordOutput("Turret/ShooterDistanceMeters", observation.shooterDistanceMeters);
    Logger.recordOutput("Turret/RobotBearingDegrees", observation.robotBearingDegrees);
    Logger.recordOutput("Turret/DistanceSource", observation.distanceSource);
    Logger.recordOutput("Turret/TurretReachable", observation.isWithinTurretRange());
    Logger.recordOutput("Turret/OnTarget", turret.isOnTarget());

    if (!observation.isWithinTurretRange()) {
      maybeWarnOutOfRange(observation);
    }

    maybeLog("TRACKING", observation, currentAngleDegrees, commandedDegrees);
  }

  /**
   * Rate-limited logging.
   *
   * <p>The previous version called DriverStation.reportWarning on every single loop, at 50 Hz, for
   * both the tracking line and the no-target line. That floods the driver station log, bloats the
   * match log, and costs real bandwidth over the radio during a match.
   */
  private void maybeLog(
      String state,
      TargetObservation observation,
      double currentAngleDegrees,
      double commandedDegrees) {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastLogTimestampSeconds >= 0.0 && nowSeconds - lastLogTimestampSeconds < LOG_PERIOD_SECONDS) {
      return;
    }
    lastLogTimestampSeconds = nowSeconds;

    System.out.printf(
        "TURRET_TRACK,%s,time=%.3f,tag=%d,target_deg=%.2f,current_deg=%.2f,commanded_deg=%.2f,shot_m=%.2f,src=%s%n",
        state,
        nowSeconds,
        observation.tagId,
        observation.valid ? observation.turretAngleDegrees : 0.0,
        currentAngleDegrees,
        commandedDegrees,
        observation.shooterDistanceMeters,
        observation.distanceSource);
  }

  private double lastNoTargetWarningSeconds = -1.0;

  /**
   * Says WHY there is no target, rather than just that there isn't one.
   *
   * <p>"No target" has two completely different causes with completely different fixes: the camera
   * genuinely sees nothing (aim it, check exposure, check the pipeline), or it sees tags that are
   * not in the set we accept (wrong alliance, wrong tag IDs, a practice tag on the shop floor).
   * Printing the visible IDs alongside the accepted ones makes that a one-glance diagnosis.
   */
  private void reportWhyNoTarget() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastNoTargetWarningSeconds >= 0.0 && nowSeconds - lastNoTargetWarningSeconds < 2.0) {
      return;
    }
    lastNoTargetWarningSeconds = nowSeconds;

    int[] visible = limelight.getVisibleTagIds();
    Set<Integer> allowed = allowedTagIds.get();

    if (visible.length == 0) {
      DriverStation.reportWarning(
          "[VISION] Camera sees NO AprilTags at all. Check that it is powered, on the AprilTag "
              + "pipeline, pointed at a tag, and not over-exposed.",
          false);
      return;
    }

    StringBuilder seen = new StringBuilder();
    for (int i = 0; i < visible.length; i++) {
      seen.append(i == 0 ? "" : ", ").append(visible[i]);
    }
    DriverStation.reportWarning(
        String.format(
            "[VISION] Camera sees tag(s) [%s] but none are accepted. Accepting %s for alliance %s. "
                + "Wrong alliance or wrong tag IDs.",
            seen,
            allowed,
            FieldConstants.alliance()),
        false);
  }

  private double lastOutOfRangeWarningSeconds = -1.0;

  private void maybeWarnOutOfRange(TargetObservation observation) {
    double nowSeconds = Timer.getFPGATimestamp();
    if (lastOutOfRangeWarningSeconds >= 0.0 && nowSeconds - lastOutOfRangeWarningSeconds < 1.0) {
      return;
    }
    lastOutOfRangeWarningSeconds = nowSeconds;
    DriverStation.reportWarning(
        String.format(
            "[TURRET] Target needs %.1f deg, outside the +/-%.0f deg turret range (tag %d). Rotate the chassis.",
            observation.turretAngleDegrees,
            TurretConstants.MAX_ONE_DIR_FOV_DEGREES,
            observation.tagId),
        false);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setOpenLoop(0.0);
    turret.clearTrackingTelemetry();
    turret.setTrackingError(Double.POSITIVE_INFINITY);
    limelight.clearTagFilter();
    Logger.recordOutput("Turret/OnTarget", false);
  }
}
