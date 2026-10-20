package frc.robot.commands;

import java.util.Optional;


import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.vision.VisionTarget;

/**
 * Points the turret at whatever tag the camera has locked onto.
 *
 * <h2>The two rules this is built around</h2>
 *
 * <p><b>No target means no movement. Ever.</b> If the camera has no lock, output is zero and the
 * turret is still. There is no coasting to a remembered angle: the previous version kept driving
 * toward the last aim point through a dropout, which is literally "the turret moves on its own
 * with no tag detected". A turret that drifts by itself is worse than one that stops.
 *
 * <p><b>tx is not the turret angle.</b> The camera is bolted to the CHASSIS, 8.5 inches right of
 * the turret pivot, and never moves. So tx is the angle from the LENS, and the turret pivot sees
 * the same tag at a different angle — about 6 degrees apart at 2 m, over 12 at 1 m. The tag is
 * resolved to a point in the robot frame and the angle is taken from the pivot to that point.
 *
 * <p>Because the camera is fixed, that solved angle does not change as the turret swings: it is a
 * stationary setpoint the turret simply drives to. Nothing feeds the turret's own position back
 * into its own setpoint, so there is no loop to hunt.
 */
public class TurretAimCommand extends Command {

  /** Smoothing on the solved angle. Higher tracks faster and shakes more. */
  private static final double FILTER_ALPHA = 0.35;

  /** Error under this counts as on-target for readiness reporting. */
  private static final double ON_TARGET_DEADBAND_DEGREES = 0.25;

  private static final double LOG_PERIOD_SECONDS = 0.5;
  private static final double WARN_PERIOD_SECONDS = 3.0;

  private final TurretSubsystem turret;
  private final Limelight camera;

  private double filteredAngleDegrees = 0.0;
  private int lastTagId = -1;
  private boolean hasSample = false;
  private double lastLogSeconds = -1.0;
  private double lastWarnSeconds = -1.0;

  public TurretAimCommand(TurretSubsystem turret, Limelight camera) {
    this.turret = turret;
    this.camera = camera;
    addRequirements(turret);
    setName("Turret.aim");
  }

  @Override
  public void initialize() {
    hasSample = false;
    lastTagId = -1;
    lastLogSeconds = -1.0;
    lastWarnSeconds = -1.0;
    turret.clearTrackingTelemetry();
  }

  @Override
  public void execute() {
    // A HUB tag wins over anything else in view.
    //
    // The trench is a structure covered in AprilTags, and every auto parks beside it to shoot, so
    // a trench tag is routinely the nearest and squarest thing the camera can see. Taking the
    // plain locked target there aims the turret at the tunnel wall.
    //
    // The fallback to getTarget() is deliberate rather than tidy: with no hub in view the turret
    // still tracks whatever tag it has, which is what makes bench testing with a tag held up in
    // front of the robot work at all.
    Optional<VisionTarget> target = camera.getHubTarget();
    if (target.isEmpty()) {
      target = camera.getTarget();
    }

    if (target.isEmpty()) {
      stopAndReportIdle();
      return;
    }

    VisionTarget tag = target.get();
    double currentDegrees = turret.getRawAngle().in(Degrees);
    double solvedDegrees = solveTurretAngle(tag);

    // A different tag is a discontinuous aim point; re-seed instead of filtering across the jump.
    if (tag.id != lastTagId) {
      hasSample = false;
      lastTagId = tag.id;
    }

    if (!hasSample) {
      filteredAngleDegrees = solvedDegrees;
      hasSample = true;
    } else {
      filteredAngleDegrees += (solvedDegrees - filteredAngleDegrees) * FILTER_ALPHA;
    }

    // No glitch rejector here on purpose. The camera already drops implausible readings and
    // debounces its lock, so anything arriving now is trustworthy. The old rejector could latch
    // forever on a persistently noisy reading and freeze the aim point, which is what produced a
    // long random delay before the turret did anything.
    double commanded = MathUtil.clamp(
        filteredAngleDegrees,
        -TurretConstants.MAX_ONE_DIR_FOV_DEGREES,
        TurretConstants.MAX_ONE_DIR_FOV_DEGREES);

    turret.driveTowardAngle(commanded);

    double error = solvedDegrees - currentDegrees;
    if (Math.abs(error) < ON_TARGET_DEADBAND_DEGREES) {
      error = 0.0;
    }
    turret.setTrackingError(error);
    turret.updateTrackingTelemetry(tag.id, error, 0.0);

    frc.robot.util.RobotLog.put("Turret/AimState", "AIMING");
    frc.robot.util.RobotLog.put("Turret/AimTagId", tag.id);
    frc.robot.util.RobotLog.put("Turret/AimSolvedDegrees", solvedDegrees);
    frc.robot.util.RobotLog.put("Turret/AimCommandedDegrees", commanded);
    frc.robot.util.RobotLog.put("Turret/AimActualDegrees", currentDegrees);
    frc.robot.util.RobotLog.put("Turret/AimErrorDegrees", error);
    frc.robot.util.RobotLog.put("Turret/AimRangeMeters", tag.rangeMeters);
    frc.robot.util.RobotLog.put("Turret/AimOnTarget", turret.isOnTarget());

    // Where the solver thinks the tag is, in the robot frame. Publishing it makes a mounting-sign
    // error visible directly: park square-on to a tag and TargetY should be near zero. If it sits
    // at roughly twice the lens offset, the lateral mounting sign is backwards.
    frc.robot.util.RobotLog.put("Turret/TargetXMeters", tag.robotFrameX());
    frc.robot.util.RobotLog.put("Turret/TargetYMeters", tag.robotFrameY());
    frc.robot.util.RobotLog.put("Turret/TargetYInches", tag.robotFrameY() * 39.3701);

    if (Math.abs(solvedDegrees) > TurretConstants.MAX_ONE_DIR_FOV_DEGREES) {
      warnOutOfRange(solvedDegrees, tag.id);
    }
    maybeLog(tag, currentDegrees, solvedDegrees, commanded);
  }

  private void stopAndReportIdle() {
    hasSample = false;
    lastTagId = -1;
    turret.setOpenLoop(0.0);
    turret.setTrackingError(Double.POSITIVE_INFINITY);
    turret.clearTrackingTelemetry();
    frc.robot.util.RobotLog.put("Turret/AimState", "NO_TARGET");
    frc.robot.util.RobotLog.put("Turret/AimOnTarget", false);

    double now = Timer.getFPGATimestamp();
    if (lastWarnSeconds < 0.0 || now - lastWarnSeconds >= WARN_PERIOD_SECONDS) {
      lastWarnSeconds = now;
      DriverStation.reportWarning(
          "[TURRET] Holding still: no locked target. Camera says: " + camera.getStatus(), false);
    }
  }

  /**
   * Absolute turret angle that points the shooter at the tag.
   *
   * <p>Places the tag in the robot frame from the lens position and the measured range, then takes
   * the angle from the turret pivot to that point. The range is already validated by the camera,
   * so there is no unusable-range branch and no silent fallback to an uncorrected bearing — the
   * two used to differ by the full parallax and flip-flopping between them made the solved angle
   * swing wildly.
   */
  private double solveTurretAngle(VisionTarget tag) {
    double pivotX = TurretConstants.PIVOT_TO_ROBOT_CENTER.getX();
    double pivotY = TurretConstants.PIVOT_TO_ROBOT_CENTER.getY();

    // Direction from the turret pivot to the tag, in the robot frame (0 = robot forward, CCW+).
    double robotFrameDirection = Math.toDegrees(
        Math.atan2(tag.robotFrameY() - pivotY, tag.robotFrameX() - pivotX));

    // Convert that into an ENCODER angle. The turret is mounted at the back, so its zero points
    // out the rear: a tag directly behind the robot is robot-frame 180, which must come out as
    // turret 0. Normalising afterwards keeps the result in (-180, 180] so the +/-90 soft limits
    // compare against something sane instead of, say, -270.
    return normalize(
        robotFrameDirection
            - TurretConstants.ZERO_ROBOT_DIRECTION_DEGREES
            + TurretConstants.ENCODER_OFFSET_DEGREES);
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

  private void warnOutOfRange(double wantedDegrees, int tagId) {
    double now = Timer.getFPGATimestamp();
    if (lastWarnSeconds >= 0.0 && now - lastWarnSeconds < WARN_PERIOD_SECONDS) {
      return;
    }
    lastWarnSeconds = now;
    DriverStation.reportWarning(
        String.format(
            "[TURRET] Tag %d needs %.1f deg, outside the +/-%.0f deg turret range. Rotate the "
                + "chassis.",
            tagId, wantedDegrees, TurretConstants.MAX_ONE_DIR_FOV_DEGREES),
        false);
  }

  private void maybeLog(
      VisionTarget tag, double currentDegrees, double solvedDegrees, double commandedDegrees) {
    double now = Timer.getFPGATimestamp();
    if (lastLogSeconds >= 0.0 && now - lastLogSeconds < LOG_PERIOD_SECONDS) {
      return;
    }
    lastLogSeconds = now;
    System.out.printf(
        "TURRET_AIM,tag=%d,tx=%.2f,range=%.2fm(%s),solved=%.2f,actual=%.2f,commanded=%.2f%n",
        tag.id, tag.txDegrees, tag.rangeMeters, tag.rangeSource,
        solvedDegrees, currentDegrees, commandedDegrees);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setOpenLoop(0.0);
    turret.setTrackingError(Double.POSITIVE_INFINITY);
    turret.clearTrackingTelemetry();
    frc.robot.util.RobotLog.put("Turret/AimState", "IDLE");
    frc.robot.util.RobotLog.put("Turret/AimOnTarget", false);
  }
}
