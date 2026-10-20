package frc.robot.commands;

import java.util.Optional;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.vision.VisionTarget;

/**
 * Yaws the chassis to centre on a tag, and keeps it centred as things move.
 *
 * <h2>Which way "centred" points</h2>
 *
 * <p>The camera and the turret both face out the BACK of the robot, so centring means putting the
 * tag on the robot's REAR centreline, not the front. The target bearing is therefore
 * {@link TurretConstants#ZERO_ROBOT_DIRECTION_DEGREES} (180) rather than 0. Squaring the front of
 * the robot up on the tag would point the shooter directly away from it.
 *
 * <h2>Centring the robot, not the camera</h2>
 *
 * <p>The error comes from {@link VisionTarget#bearingFromRobotCenterDegrees()}, not from tx. The
 * lens sits 8.5 inches off the robot's centreline and never moves, so driving tx to zero squares
 * the CAMERA up on the tag and leaves the robot itself pointing a few degrees off — about 6
 * degrees at 2 m, over 12 at 1 m.
 *
 * <h2>Held button only</h2>
 *
 * <p>This owns the drivebase while it runs and never finishes on its own, so it must be bound with
 * {@code whileTrue} and must never be a default command. Releasing the button hands the robot
 * straight back to the driver. With no locked target it commands zero and stays still rather than
 * drifting on a remembered heading.
 */
public class DriveAlignCommand extends Command {

  /** Radians per second of yaw per degree of bearing error. */
  private static final double ROTATION_KP = 0.045;

  private static final double MAX_OMEGA_RAD_PER_SEC = 2.0;

  /** Yaw acceleration limit, so the chassis eases in rather than snapping. */
  private static final double OMEGA_SLEW_RAD_PER_SEC_SQ = 6.0;

  /** Below this the robot counts as centred and yaw stops. */
  private static final double BEARING_DEADBAND_DEGREES = 1.5;

  // ---- Range control --------------------------------------------------------

  /** Standoff the robot drives to once it is pointed at the tag. 7.5 ft. */
  public static final double TARGET_RANGE_METERS = edu.wpi.first.math.util.Units
      .feetToMeters(7.5);

  /** Range error under this counts as arrived. About 4 inches. */
  private static final double RANGE_DEADBAND_METERS = 0.10;

  /** Metres per second per metre of range error. */
  private static final double RANGE_KP = 1.4;

  private static final double MAX_DRIVE_METERS_PER_SEC = 1.2;
  private static final double DRIVE_SLEW_METERS_PER_SEC_SQ = 2.0;

  /** Smallest translation command that actually moves the chassis. */
  private static final double MIN_DRIVE_METERS_PER_SEC = 0.15;

  /**
   * Do not close range until the bearing is inside this.
   *
   * <p>Turn first, then drive. Driving while badly misaimed sweeps a long arc, which both takes
   * longer and risks swinging the tag out of the camera's view entirely.
   */
  private static final double TURN_FIRST_BEARING_DEGREES = 12.0;

  /** Smallest yaw command that actually turns the chassis, applied outside the deadband. */
  private static final double MIN_OMEGA_RAD_PER_SEC = 0.12;

  private static final double LOG_PERIOD_SECONDS = 0.5;
  private static final double WARN_PERIOD_SECONDS = 3.0;

  private final SwerveSubsystem drivebase;
  private final Limelight camera;
  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(OMEGA_SLEW_RAD_PER_SEC_SQ);
  private final SlewRateLimiter driveLimiter =
      new SlewRateLimiter(DRIVE_SLEW_METERS_PER_SEC_SQ);

  private double lastLogSeconds = -1.0;
  private double lastWarnSeconds = -1.0;

  public DriveAlignCommand(SwerveSubsystem drivebase, Limelight camera) {
    this.drivebase = drivebase;
    this.camera = camera;
    addRequirements(drivebase);
    setName("Drive.alignToTag");
  }

  @Override
  public void initialize() {
    omegaLimiter.reset(0.0);
    driveLimiter.reset(0.0);
    lastLogSeconds = -1.0;
    lastWarnSeconds = -1.0;
  }

  @Override
  public void execute() {
    Optional<VisionTarget> target = camera.getTarget();

    if (target.isEmpty()) {
      // No lock means no motion. Same rule as the turret: a drivebase that keeps yawing on a
      // remembered heading is worse than one that stops.
      drivebase.drive(new Translation2d(driveLimiter.calculate(0.0), 0.0),
          omegaLimiter.calculate(0.0), false);
      frc.robot.util.RobotLog.put("DriveAlign/State", "NO_TARGET");
      frc.robot.util.RobotLog.put("DriveAlign/Omega", 0.0);
      warnNoTarget();
      return;
    }

    VisionTarget tag = target.get();

    // Error between where the tag is and where we want it: on the robot's rear centreline.
    double errorDegrees = normalize(
        tag.bearingFromRobotCenterDegrees() - TurretConstants.ZERO_ROBOT_DIRECTION_DEGREES);

    boolean centred = Math.abs(errorDegrees) <= BEARING_DEADBAND_DEGREES;
    double omega = 0.0;

    if (!centred) {
      omega = errorDegrees * ROTATION_KP;
      if (Math.abs(omega) < MIN_OMEGA_RAD_PER_SEC) {
        omega = Math.copySign(MIN_OMEGA_RAD_PER_SEC, omega);
      }
      omega = MathUtil.clamp(omega, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);
    }

    // ---- Range: once pointed at the tag, drive to the standoff distance ----
    //
    // The camera and turret face out the BACK of the robot, so the tag being "ahead" of the
    // shooter means it is BEHIND the chassis. Closing range therefore means driving in -X, and
    // the sign below reflects that rather than a sign error.
    double rangeMeters = tag.rangeFromRobotCenterMeters();
    double rangeError = rangeMeters - TARGET_RANGE_METERS;
    boolean atRange = Math.abs(rangeError) <= RANGE_DEADBAND_METERS;

    double forward = 0.0;
    if (!atRange && Math.abs(errorDegrees) <= TURN_FIRST_BEARING_DEGREES) {
      // Positive rangeError means too far away; the tag is behind us, so close the gap by
      // driving backwards.
      double demand = -rangeError * RANGE_KP;
      if (Math.abs(demand) < MIN_DRIVE_METERS_PER_SEC) {
        demand = Math.copySign(MIN_DRIVE_METERS_PER_SEC, demand);
      }
      forward = MathUtil.clamp(demand, -MAX_DRIVE_METERS_PER_SEC, MAX_DRIVE_METERS_PER_SEC);
    }

    omega = omegaLimiter.calculate(omega);
    forward = driveLimiter.calculate(forward);
    drivebase.drive(new Translation2d(forward, 0.0), omega, false);

    boolean lined = centred && atRange;
    String state = lined ? "READY" : (centred ? "CLOSING RANGE" : "TURNING");

    frc.robot.util.RobotLog.put("DriveAlign/State", state);
    frc.robot.util.RobotLog.put("DriveAlign/TagId", tag.id);
    frc.robot.util.RobotLog.put("DriveAlign/BearingErrorDegrees", errorDegrees);
    frc.robot.util.RobotLog.put("DriveAlign/RangeMeters", rangeMeters);
    frc.robot.util.RobotLog.put("DriveAlign/RangeFeet", rangeMeters * 3.28084);
    frc.robot.util.RobotLog.put("DriveAlign/RangeErrorFeet", rangeError * 3.28084);
    frc.robot.util.RobotLog.put("DriveAlign/Forward", forward);
    frc.robot.util.RobotLog.put("DriveAlign/Omega", omega);
    frc.robot.util.RobotLog.put("DriveAlign/Centered", centred);
    frc.robot.util.RobotLog.put("DriveAlign/AtRange", atRange);
    frc.robot.util.RobotLog.put("DriveAlign/LinedUp", lined);

    maybeLog(tag, errorDegrees, omega);
  }

  private void warnNoTarget() {
    double now = Timer.getFPGATimestamp();
    if (lastWarnSeconds >= 0.0 && now - lastWarnSeconds < WARN_PERIOD_SECONDS) {
      return;
    }
    lastWarnSeconds = now;
    DriverStation.reportWarning(
        "[DRIVE ALIGN] Holding still: no locked target. Camera says: " + camera.getStatus(),
        false);
  }

  private void maybeLog(VisionTarget tag, double errorDegrees, double omega) {
    double now = Timer.getFPGATimestamp();
    if (lastLogSeconds >= 0.0 && now - lastLogSeconds < LOG_PERIOD_SECONDS) {
      return;
    }
    lastLogSeconds = now;
    System.out.printf(
        "DRIVE_ALIGN,tag=%d,tx=%.2f,bearing=%.2f,error=%.2f,range=%.2f,omega=%.3f%n",
        tag.id,
        tag.txDegrees,
        tag.bearingFromRobotCenterDegrees(),
        errorDegrees,
        tag.rangeFromRobotCenterMeters(),
        omega);
  }

  /** Wraps an angle into (-180, 180]. */
  private static double normalize(double degrees) {
    double wrapped = degrees % 360.0;
    if (wrapped > 180.0) {
      wrapped -= 360.0;
    } else if (wrapped <= -180.0) {
      wrapped += 360.0;
    }
    return wrapped;
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(Translation2d.kZero, 0.0, false);
    frc.robot.util.RobotLog.put("DriveAlign/State", "IDLE");
    frc.robot.util.RobotLog.put("DriveAlign/Omega", 0.0);
  }
}
