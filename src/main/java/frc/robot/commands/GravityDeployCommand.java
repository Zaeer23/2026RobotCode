package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.RobotLog;

/**
 * Drops the intake pivot on a temporary printed gear, using gravity with a little motor help.
 *
 * <p>The CyberPsycho contingency. The printed gear can carry some torque, just far less than the
 * real one, so this is not the pure coast-and-pray version: the motor pushes the arm down the
 * whole time, the chassis is shaken to break it loose, and the rollers add reaction torque. What
 * keeps the gear alive is the current limit, not restraint in the duty cycle.
 *
 * <p><b>The current limit is the safety mechanism.</b> Current is torque. A NEO makes 2.6 Nm at
 * its 105 A stall, so the normal 40 A arrives at the arm as roughly 49 Nm, which would shear a
 * printed gear. This drops the limit to {@link IntakeSubsystem#PIVOT_LIMP_CURRENT_AMPS} for the
 * whole sequence — about 7 Nm at the arm — and restores nothing on the way out, because the gear
 * is still printed after the command ends. Only putting the real gear back should raise it.
 *
 * <p>Three things it does beyond pushing:
 *
 * <ul>
 *   <li><b>It alternates the chassis sharply rather than smoothly.</b> What shifts a stuck arm is
 *       the reversal, not the speed — a robot gliding at constant velocity does nothing to it.
 *       Beats are short to keep the jerk high.</li>
 *   <li><b>It twists every other pair of beats.</b> The arm hangs on one axis, so a pure
 *       fore-and-aft shake only ever loads it one way. A yaw kick attacks a pivot that is wedged
 *       rather than merely stiff.</li>
 *   <li><b>It eases off as the arm comes down.</b> Full limp output is for breaking stiction at
 *       the top. Once the arm is swinging, gravity is doing the work and the motor backs down to
 *       a hold, so the gear is not taking its biggest load at the bottom of the travel where the
 *       arm arrives with momentum.</li>
 * </ul>
 *
 * <p>It ends on the encoder reading deployed, or on a timeout. The timeout is the one that
 * matters: if the printed gear strips, the encoder is reading the MOTOR, which is no longer
 * attached to the arm, and will happily report that it arrived when it did not. Nothing here
 * depends on believing it.
 */
public class GravityDeployCommand extends Command {

  /** How long one half of a shake lasts. Short on purpose: the reversal does the work. */
  private static final double BEAT_SECONDS = 0.12;

  /** Chassis speed during a beat, m/s. */
  private static final double SHAKE_SPEED = 2.2;

  /** Yaw kick on alternate beat pairs, rad/s. */
  private static final double TWIST_SPEED = 3.0;

  /** Give up after this long whether or not the arm reports itself down. */
  private static final double TIMEOUT_SECONDS = 2.5;

  /** Below this angle the arm counts as deployed. */
  private static final double DEPLOYED_DEGREES = 12.0;

  /** Above this the arm is still near the top and needs the harder push to break stiction. */
  private static final double STICTION_DEGREES = 45.0;

  /** Duty cycle while breaking the arm loose at the top. Bounded by the current limit. */
  private static final double BREAKAWAY_OUTPUT = 0.45;

  /** Duty cycle once it is moving, when gravity is doing the work. */
  private static final double FOLLOW_OUTPUT = 0.15;

  private final SwerveSubsystem drivebase;
  private final IntakeSubsystem intake;
  private final Timer timer = new Timer();

  public GravityDeployCommand(SwerveSubsystem drivebase, IntakeSubsystem intake) {
    this.drivebase = drivebase;
    this.intake = intake;
    addRequirements(drivebase, intake);
  }

  @Override
  public void initialize() {
    // Cap the torque BEFORE anything is commanded. Doing it the other way round means the first
    // loop of full-current push goes through the printed gear.
    intake.setPivotCurrentLimit(IntakeSubsystem.PIVOT_LIMP_CURRENT_AMPS);
    // Coast, so that whenever nothing is commanding the pivot the gear is carrying nothing.
    intake.setPivotCoast(true);

    timer.restart();
    RobotLog.put("IntakePivot/GravityDeployActive", true);
    System.out.printf("[PIVOT] Gravity deploy: limited to %d A, pushing down and shaking.%n",
        IntakeSubsystem.PIVOT_LIMP_CURRENT_AMPS);
  }

  @Override
  public void execute() {
    int beat = (int) (timer.get() / BEAT_SECONDS);

    // Square wave, not a sine: slam one way, slam the other.
    double direction = (beat % 2 == 0) ? 1.0 : -1.0;
    double twist = ((beat / 2) % 2 == 0) ? TWIST_SPEED * direction : 0.0;
    drivebase.drive(new Translation2d(SHAKE_SPEED * direction, 0.0), twist, false);

    // Push hardest while the arm is still up and stuck; ease off once it is falling, so the gear
    // is not carrying its peak load just as the arm arrives at the bottom with momentum.
    double push = intake.pivotAngleDegrees() > STICTION_DEGREES ? BREAKAWAY_OUTPUT : FOLLOW_OUTPUT;
    intake.runLimpPivot(-push);

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
    intake.runLimpPivot(0.0);
    timer.stop();

    // The current limit is deliberately NOT restored. The gear is still printed after this
    // command ends, and every other thing that moves the pivot would otherwise go straight back
    // to 40 A. It stays limited until someone fits the real gear and restarts the robot.
    double angle = intake.pivotAngleDegrees();
    RobotLog.put("IntakePivot/GravityDeployActive", false);
    RobotLog.put("IntakePivot/GravityDeployEndAngle", angle);
    System.out.printf(
        "[PIVOT] Gravity deploy finished after %.2f s, encoder reads %.1f deg%s%n",
        timer.get(), angle,
        angle <= DEPLOYED_DEGREES ? "" : " (did not reach deployed - gear may have stripped)");
  }
}
