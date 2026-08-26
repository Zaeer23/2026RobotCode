package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoLineupConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.VisionTargeting.TargetObservation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Drives the chassis to a good shooting position on the HUB.
 *
 * <p>Two jobs, both robot-relative:
 *
 * <ul>
 *   <li><b>Yaw</b> until the hub is straight ahead. This is not about aiming — the turret does that
 *       — it is about keeping the hub inside the turret's +/-90 degree window with margin to spare,
 *       so a turret that is already near a soft limit gets its authority back.
 *   <li><b>Range</b> until the robot sits at the calibrated standoff distance, where the shot table
 *       has the most data.
 * </ul>
 *
 * <p>Range is measured in meters from the vision solver, not from the tag's apparent area. Area
 * falls off as roughly the inverse square of distance and is extremely sensitive to partial tag
 * occlusion, so an area setpoint is a different physical distance depending on viewing angle.
 *
 * <p><b>This is a held-button command by design.</b> It takes over the drivebase completely, so it
 * must never be a default command — the driver would never be able to move.
 */
public class HubLineupCommand extends Command {

  private final SwerveSubsystem drivebase;
  private final LimeLight limelight;

  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(AutoLineupConstants.OMEGA_SLEW_RAD_PER_SEC_SQ);
  private final SlewRateLimiter forwardLimiter =
      new SlewRateLimiter(AutoLineupConstants.FORWARD_SLEW_METERS_PER_SEC_SQ);

  private int settledCycles = 0;
  private int missingCycles = 0;
  private boolean gaveUp = false;

  public HubLineupCommand(SwerveSubsystem drivebase, LimeLight limelight) {
    this.drivebase = drivebase;
    this.limelight = limelight;
    addRequirements(drivebase);
    setName("HubLineup");
  }

  @Override
  public void initialize() {
    settledCycles = 0;
    missingCycles = 0;
    gaveUp = false;
    omegaLimiter.reset(0.0);
    forwardLimiter.reset(0.0);
    limelight.setTagFilter(FieldConstants.hubTagIds());
  }

  @Override
  public void execute() {
    TargetObservation observation = limelight.observeHub(drivebase);

    if (!observation.valid) {
      missingCycles++;
      settledCycles = 0;
      // Coast the commanded velocity down through the limiters instead of hard-zeroing, so a
      // one-frame dropout does not jolt the chassis.
      driveRobotRelative(forwardLimiter.calculate(0.0), omegaLimiter.calculate(0.0));

      if (missingCycles >= AutoLineupConstants.MAX_MISSING_CYCLES) {
        gaveUp = true;
        DriverStation.reportWarning(
            "[LINEUP] Gave up: no HUB AprilTag visible. Point the camera at the hub and retry.",
            false);
      }
      Logger.recordOutput("Lineup/State", "NO_TARGET");
      Logger.recordOutput("Lineup/MissingCycles", missingCycles);
      return;
    }

    missingCycles = 0;

    double bearingErrorDegrees = observation.robotBearingDegrees;
    double rangeErrorMeters =
        observation.robotDistanceMeters - AutoLineupConstants.TARGET_DISTANCE_METERS;

    boolean bearingInTolerance =
        Math.abs(bearingErrorDegrees) <= AutoLineupConstants.BEARING_TOLERANCE_DEGREES;
    boolean rangeInTolerance =
        Math.abs(rangeErrorMeters) <= AutoLineupConstants.DISTANCE_TOLERANCE_METERS;

    // Bearing is CCW-positive and we want it at zero, so the correction carries its sign directly.
    double requestedOmega = MathUtil.clamp(
        bearingErrorDegrees * AutoLineupConstants.ROTATION_KP,
        -AutoLineupConstants.MAX_OMEGA_RAD_PER_SEC,
        AutoLineupConstants.MAX_OMEGA_RAD_PER_SEC);

    // Positive range error means too far away, so drive forward (+X).
    double requestedForward = MathUtil.clamp(
        rangeErrorMeters * AutoLineupConstants.FORWARD_KP,
        -AutoLineupConstants.MAX_FORWARD_METERS_PER_SEC,
        AutoLineupConstants.MAX_FORWARD_METERS_PER_SEC);

    // Turn first. Driving forward while badly misaimed sweeps a long arc and can lose the tag.
    if (Math.abs(bearingErrorDegrees) > AutoLineupConstants.TURN_ONLY_BEARING_DEGREES) {
      requestedForward = 0.0;
    }
    if (rangeInTolerance) {
      requestedForward = 0.0;
    }
    if (bearingInTolerance) {
      requestedOmega = 0.0;
    }

    double omega = omegaLimiter.calculate(requestedOmega);
    double forward = forwardLimiter.calculate(requestedForward);
    driveRobotRelative(forward, omega);

    if (bearingInTolerance && rangeInTolerance) {
      settledCycles++;
    } else {
      settledCycles = 0;
    }

    Logger.recordOutput("Lineup/State", settledCycles > 0 ? "SETTLING" : "DRIVING");
    Logger.recordOutput("Lineup/TagID", observation.tagId);
    Logger.recordOutput("Lineup/BearingErrorDegrees", bearingErrorDegrees);
    Logger.recordOutput("Lineup/RangeErrorMeters", rangeErrorMeters);
    Logger.recordOutput("Lineup/RobotDistanceMeters", observation.robotDistanceMeters);
    Logger.recordOutput("Lineup/CommandedOmega", omega);
    Logger.recordOutput("Lineup/CommandedForward", forward);
    Logger.recordOutput("Lineup/SettledCycles", settledCycles);
    Logger.recordOutput("Lineup/TurretAngleDegrees", observation.turretAngleDegrees);
    Logger.recordOutput("Lineup/TurretReachable", observation.isWithinTurretRange());
  }

  private void driveRobotRelative(double forwardMetersPerSecond, double omegaRadPerSecond) {
    drivebase.drive(new Translation2d(forwardMetersPerSecond, 0.0), omegaRadPerSecond, false);
  }

  @Override
  public boolean isFinished() {
    return gaveUp || settledCycles >= AutoLineupConstants.SETTLE_CYCLES;
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(Translation2d.kZero, 0.0, false);
    limelight.clearTagFilter();
    Logger.recordOutput("Lineup/State", "IDLE");
    Logger.recordOutput("Lineup/CommandedOmega", 0.0);
    Logger.recordOutput("Lineup/CommandedForward", 0.0);
  }

  /** True once the robot is squared up and at range — useful for gating a shot in autonomous. */
  public boolean isLinedUp() {
    return settledCycles >= AutoLineupConstants.SETTLE_CYCLES;
  }
}
