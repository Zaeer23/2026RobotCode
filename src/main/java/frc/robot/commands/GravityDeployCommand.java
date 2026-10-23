package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.RobotLog;

/**
 * Drops the intake pivot using gravity and inertia, for when the pivot can no longer drive itself.
 *
 * <p>The contingency for a stripped belt or gearbox. The pivot is released into COAST so gravity
 * can act on it at all, and then the whole robot is jerked back and forth: each reversal throws
 * the arm against its own inertia, which is far more force than the arm can make for itself. The
 * rollers run backwards throughout, pushing on the arm by reaction torque.
 *
 * <p>Three things this does beyond shaking:
 *
 * <ul>
 *   <li><b>It alternates sharply rather than smoothly.</b> What moves the arm is the reversal, not
 *       the speed — a robot gliding at constant velocity does nothing to it. Short hard flips
 *       maximise the jerk, so the period is deliberately brief.</li>
 *   <li><b>It adds a twist every other beat.</b> The arm hangs on one axis, and a pure
 *       fore-and-aft shake only ever loads it one way. A yaw kick attacks it from a different
 *       direction and shakes loose a pivot that is wedged rather than merely stiff.</li>
 *   <li><b>It clamps the pivot when it finishes.</b> Coast is what let the arm fall, and it is
 *       also what lets the next hard acceleration throw it straight back up. The last act is to
 *       put the controller back into brake so the arm stays where gravity left it.</li>
 * </ul>
 *
 * <p>It ends on the encoder reading deployed, or on a timeout. The timeout is the one that
 * matters: if the belt stripped, the encoder is still reading the MOTOR, which is not attached to
 * the arm any more and will happily report nothing moved — or report it arrived when it did not.
 * Never let this command's success depend on believing it.
 */
public class GravityDeployCommand extends Command {

  /** How long one half of a shake lasts. Short on purpose: the reversal is what does the work. */
  private static final double BEAT_SECONDS = 0.12;

  /** Chassis speed during a beat, m/s. */
  private static final double SHAKE_SPEED = 2.2;

  /** Yaw kick on alternate beats, rad/s. */
  private static final double TWIST_SPEED = 3.0;

  /** Give up after this long whether or not the arm reports itself down. */
  private static final double TIMEOUT_SECONDS = 2.5;

  /** Below this angle the arm is considered deployed. */
  private static final double DEPLOYED_DEGREES = 12.0;

  private final SwerveSubsystem drivebase;
  private final IntakeSubsystem intake;
  private final Timer timer = new Timer();

  private int beat;

  public GravityDeployCommand(SwerveSubsystem drivebase, IntakeSubsystem intake) {
    this.drivebase = drivebase;
    this.intake = intake;
    addRequirements(drivebase, intake);
  }

  @Override
  public void initialize() {
    // Release FIRST. Shaking a braked pivot just shakes the robot.
    intake.setPivotCoast(true);
    beat = 0;
    timer.restart();
    RobotLog.put("IntakePivot/GravityDeployActive", true);
    System.out.println("[PIVOT] Gravity deploy: released to coast, shaking.");
  }

  @Override
  public void execute() {
    int current = (int) (timer.get() / BEAT_SECONDS);
    beat = current;

    // Square wave, not a sine: slam one way, slam the other.
    double direction = (beat % 2 == 0) ? 1.0 : -1.0;
    // Twist on every other pair of beats, so the arm gets loaded off-axis too.
    double twist = ((beat / 2) % 2 == 0) ? TWIST_SPEED * direction : 0.0;

    drivebase.drive(new Translation2d(SHAKE_SPEED * direction, 0.0), twist, false);

    // Rollers push back on the arm the whole time.
    intake.runGravityAssistRollers(true);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(TIMEOUT_SECONDS)
        || intake.pivotAngleDegrees() <= DEPLOYED_DEGREES;
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0.0, false);
    intake.runGravityAssistRollers(false);
    // Clamp it down. Without this the next hard acceleration throws the arm back up.
    intake.setPivotCoast(false);
    timer.stop();

    double angle = intake.pivotAngleDegrees();
    RobotLog.put("IntakePivot/GravityDeployActive", false);
    RobotLog.put("IntakePivot/GravityDeployEndAngle", angle);
    System.out.printf(
        "[PIVOT] Gravity deploy finished after %.2f s, encoder reads %.1f deg%s%n",
        timer.get(), angle,
        angle <= DEPLOYED_DEGREES ? "" : " (did not reach deployed - encoder may be lying)");
  }
}
