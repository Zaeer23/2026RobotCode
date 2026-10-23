package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Removed ThriftyNova import
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper; // Only need SparkWrapper now

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 1.0;
  private static final double PIVOT_STOW_DEGREES = 90.0;
  private static final double PIVOT_DEPLOY_DEGREES = 0.0;
  private static final double PIVOT_MIN_DEGREES = 0.0;
  private static final double PIVOT_MAX_DEGREES = 90.0;
  private static final double PIVOT_JOYSTICK_THRESHOLD = 0.25;
  /**
   * Where the pivot sits while a shot is being fired.
   *
   * <p>Raised, so the intake is clear of the ball's path out of the shooter. Feeding with the
   * pivot down puts structure where the ball needs to be, which deflects shots unpredictably and
   * looks like a shooter accuracy problem rather than a mechanism-in-the-way problem.
   */
  private static final double PIVOT_SHOOT_DEGREES = 75.0;

  private static final double PIVOT_BOUNCE_PERIOD_SECONDS = 0.18;
  /**
   * How far above the deployed angle still counts as "coming down".
   *
   * <p>The rollers backfeed while descending so the pivot can clear the hood, and intake once it
   * is down. Without a band around the setpoint the rollers would flip direction on every small
   * oscillation as the pivot settles.
   */
  private static final double PIVOT_DOWN_CLEAR_TOLERANCE_DEGREES = 8.0;

  /**
   * Pivot output below which a manual command does not count as "driving it down".
   *
   * <p>Keeps a stick resting slightly off centre from spinning the rollers backwards forever.
   */
  private static final double PIVOT_DESCENT_OUTPUT_DEADBAND = 0.05;

  // ---- Pivot gearing -----------------------------------------------------------------------
  // Motor rotations to pivot degrees. Every commanded angle scales with this, and the soft
  // limits are in degrees, so a stale value does not read as "slightly off" -- it drives the
  // pivot into a hard stop. Re-measure with printGearingCalibrationCommand() after any gearbox
  // change rather than reasoning about it.

  private static final double PIVOT_STAGE_1 = 5.0;
  private static final double PIVOT_STAGE_2 = 5.0;

  /**
   * Final stage: the small pulley driving the big one, as driven/driver teeth.
   *
   * <p>Driven by a 600-5M-15 belt: 600 mm pitch length at 5 mm HTD pitch, so 120 teeth, 15 mm
   * wide. That fixes the centre distance and roughly the SUM of the two pulleys' teeth, but not
   * this ratio, which is their quotient — 18/60 and 24/48 both run happily on the same belt.
   *
   * <p>2.0 is an ESTIMATE, not a measurement. The stage was changed to a smaller reduction than
   * the original 60/18 = 3.333 and the new pulley teeth were not recorded, so this is reasoned
   * from the belt: reusing a 120-tooth belt at the same centres holds the SUM of the two pulleys
   * near the original 18 + 60 = 78. Pairs that fit that and are smaller than 3.333 run roughly
   * 1.6 to 2.5 — 30/48, 26/52, 24/54. 2.0 sits at the low end of that band.
   *
   * <p>Low on purpose. The two ways of being wrong are not equally bad: too HIGH and the pivot
   * travels further than commanded and stalls into the hard stop on every deploy; too LOW and it
   * stops short, which costs some intake reach and breaks nothing. If the real pair turns out to
   * be 24/54, this undershoots a 90 degree deploy by about 10 degrees.
   *
   * <p>Replace it with the measurement as soon as anyone can read the pulleys or run
   * {@link #printGearingCalibrationCommand()}. An estimate that works is still an estimate.
   */
  private static final double PIVOT_FINAL_STAGE = 2.0;

  private static final double PIVOT_GEARING = PIVOT_STAGE_1 * PIVOT_STAGE_2 * PIVOT_FINAL_STAGE;

  // Pivot oscillation is commanded as raw duty cycle, not as a position setpoint. At a 0.18-0.20s
  // period the closed-loop controller never reaches either endpoint before the target flips, so it
  // only ever integrates against itself; driving the motor open-loop actually produces the shake.
  private static final double PIVOT_BOUNCE_UP_OUTPUT = 0.55;
  private static final double PIVOT_BOUNCE_DOWN_OUTPUT = -0.25;

  // 1. Changed roller motor to SparkMax with Brushless (NEO) type
  private final SparkMax rollerMotor = new SparkMax(Constants.IntakeConstants.kRollerMotorId, MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) 
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.20))
      .withOpenLoopRampRate(Seconds.of(0.20));

  // 2. Changed NovaWrapper to SparkWrapper and NeoVortex to NEO
  private final SmartMotorController smc = new SparkWrapper(rollerMotor, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

  private final FlyWheel intake = new FlyWheel(intakeConfig);

  private final SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(25, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(360))
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(
          GearBox.fromReductionStages(PIVOT_STAGE_1, PIVOT_STAGE_2, PIVOT_FINAL_STAGE)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withSoftLimit(Degrees.of(PIVOT_MIN_DEGREES), Degrees.of(PIVOT_MAX_DEGREES))
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  // 3. Pivot was already SparkMax, but ensured DCMotor is set to NEO (standard)
  private final SparkMax pivotMotor = new SparkMax(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);

  private final SmartMotorController intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNEO(1),
      intakePivotSmartMotorConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      .withSoftLimits(Degrees.of(PIVOT_MIN_DEGREES), Degrees.of(PIVOT_MAX_DEGREES))
      .withHardLimit(Degrees.of(PIVOT_MIN_DEGREES), Degrees.of(PIVOT_MAX_DEGREES + 5.0))
      .withStartingPosition(Degrees.of(PIVOT_STOW_DEGREES))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);
      

  private final Arm intakePivot = new Arm(intakePivotConfig);
  private double joystickPivotTargetDegrees = PIVOT_STOW_DEGREES;

   public Command setPercent(Supplier<Double> percentSupplier) {
    return Commands.runOnce(() -> {}) // no requirement
        .andThen(Commands.run(() -> rollerMotor.set(percentSupplier.get())))
        .finallyDo(() -> rollerMotor.set(0));
}
  public IntakeSubsystem() {
      // Configuration is handled by the YAMS SparkWrapper
  }

  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Run");
  }

  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return intakePivot.setAngle(() -> Degrees.of(
        MathUtil.clamp(angle.in(Degrees), PIVOT_MIN_DEGREES, PIVOT_MAX_DEGREES)))
        .withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * Drops the intake and collects.
   *
   * <p>Two things this deliberately does NOT do, both learned on the robot:
   *
   * <p>It does not bounce the pivot. An earlier version oscillated it the whole time the intake
   * ran, copying the shake used while feeding. Bouncing belongs to the shot, where it shakes FUEL
   * loose into the kicker; during collection it just stops the intake sitting on the floor where
   * it can pick anything up.
   *
   * <p>It does not run the rollers forward the whole time. The pivot has to get past the hood on
   * the way down, and FUEL caught in the intake jams it against that lip. So the rollers run
   * BACKWARD while the pivot is still travelling down, clearing the throat so it can drop through,
   * and flip to intaking only once it is actually down. Reversing the whole time would spit out
   * what we just collected, so this is strictly while descending.
   */
  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      intakePivotController.setPosition(Degrees.of(PIVOT_DEPLOY_DEGREES));
      smc.setDutyCycle(isPivotDescending() ? -INTAKE_SPEED : INTAKE_SPEED);
    }, this).finallyDo(() -> {
      pivotMotor.set(0);
      smc.setDutyCycle(0);
    }).withName("Intake.DeployAndRoll");
  }

  /**
   * True while the pivot is still above its deployed position, i.e. still on its way down.
   *
   * <p>The tolerance is what stops the rollers flickering between backfeed and intake once the
   * pivot settles: without it, the smallest wobble around the setpoint reverses the rollers.
   */
  public boolean isPivotDescending() {
    return intakePivot.getAngle().in(Degrees)
        > PIVOT_DEPLOY_DEGREES + PIVOT_DOWN_CLEAR_TOLERANCE_DEGREES;
  }

  public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      smc.setDutyCycle(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
    }).withName("Intake.BackFeedAndRoll");
  }
  /**
   * Holds the intake completely inert: both motors driven to zero, every loop.
   *
   * <p>Used as the default command while {@link Constants.IntakeConstants#ENABLED} is false.
   *
   * <p>This has to actively write zeros rather than simply leaving the subsystem unbound. A SparkMax
   * keeps applying its last commanded output until something tells it otherwise, so removing the
   * bindings alone would leave the intake running at whatever it was doing when the last command
   * ended — the opposite of disabled.
   */
  public Command disabledCommand() {
    return Commands.run(() -> {
      pivotMotor.set(0);
      rollerMotor.set(0);
    }, this).ignoringDisable(true).withName("Intake.DISABLED");
  }

  /**
   * Driver control of the pivot, with the rollers clearing the way on the way down.
   *
   * <p>Whenever the commanded output is downward the rollers run backward. The pivot has to get
   * past the hood, and FUEL sitting in the throat jams against that lip and makes it very hard to
   * push down. Reversing empties the throat so the pivot drops through under its own command
   * instead of fighting a wedged ball.
   *
   * <p>Only while descending. Reversing on the way up would throw away whatever was collected,
   * and holding position is not a direction, so a stationary stick leaves the rollers alone.
   */
  public Command manualPivot(DoubleSupplier pSupplier) {
    return Commands.run(() -> {
        double input = pSupplier.getAsDouble();
        double squared = Math.copySign(input * input, input);
        double output = squared * 0.5; // cap at 50% to reduce aggression
        pivotMotor.set(output);
        smc.setDutyCycle(output < -PIVOT_DESCENT_OUTPUT_DEADBAND ? -INTAKE_SPEED : 0.0);
    }, this)
        .finallyDo(() -> {
          pivotMotor.set(0);
          smc.setDutyCycle(0);
        })
        .withName("IntakePivot.Manual");
}

  /**
   * Two-position pivot on a stick, with the same descent assist as {@link #manualPivot}.
   *
   * <p>Here "descending" is whether the pivot is still above the target it was sent to, rather
   * than which way the stick is pushed: the stick only picks a target and then the controller
   * drives to it, so the stick is usually back at centre while the pivot is still travelling.
   */
  public Command joystickPivotTwoPosition(DoubleSupplier pSupplier) {
    return Commands.run(() -> {
      double input = pSupplier.getAsDouble();
      if (input > PIVOT_JOYSTICK_THRESHOLD) {
        joystickPivotTargetDegrees = PIVOT_MIN_DEGREES;
      } else if (input < -PIVOT_JOYSTICK_THRESHOLD) {
        joystickPivotTargetDegrees = PIVOT_STOW_DEGREES;
      }
      intakePivotController.setPosition(Degrees.of(joystickPivotTargetDegrees));

      boolean headingDown = joystickPivotTargetDegrees <= PIVOT_MIN_DEGREES + 1.0;
      boolean stillAbove = intakePivot.getAngle().in(Degrees)
          > joystickPivotTargetDegrees + PIVOT_DOWN_CLEAR_TOLERANCE_DEGREES;
      smc.setDutyCycle(headingDown && stillAbove ? -INTAKE_SPEED : 0.0);
    }, this)
        .finallyDo(() -> smc.setDutyCycle(0))
        .withName("IntakePivot.JoystickTwoPosition");
  }

  /**
   * Holds the pivot still at the deployed angle while FUEL is fed through the intake.
   *
   * <p>The counterpart to pivotBounceWhileFeedingCommand, which belongs to the shot. This is the
   * grab-and-keep feed: shaking here throws collected FUEL back out, which is the opposite of
   * what the operator is holding the button for.
   */
  public Command holdForFeedCommand() {
    return run(() -> intakePivotController.setPosition(Degrees.of(PIVOT_DEPLOY_DEGREES)))
        .withName("IntakePivot.HoldForFeed");
  }

  public Command pivotBounceWhileFeedingCommand() {
    return Commands.run(() -> {
      double phaseSeconds = Timer.getFPGATimestamp() % PIVOT_BOUNCE_PERIOD_SECONDS;
      boolean highPhase = phaseSeconds < (PIVOT_BOUNCE_PERIOD_SECONDS / 2.0);
      pivotMotor.set(highPhase ? PIVOT_BOUNCE_UP_OUTPUT : PIVOT_BOUNCE_DOWN_OUTPUT);
    }, this).finallyDo(() -> pivotMotor.set(0))
        .withName("IntakePivot.BounceWhileFeeding");
  }

  private void setIntakeStow() {
    intakePivotController.setPosition(Degrees.of(PIVOT_STOW_DEGREES));
  }

  private void setIntakeFeed() {
    intakePivotController.setPosition(Degrees.of(PIVOT_DEPLOY_DEGREES));
  }

  private void setIntakeHold() {
    intakePivotController.setPosition(Degrees.of(PIVOT_STOW_DEGREES));
  }

  private void setIntakeDeployed() {
    intakePivotController.setPosition(Degrees.of(PIVOT_DEPLOY_DEGREES));
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();

    // Raw motor rotations, which the gearing calibration below is read from. The mechanism angle
    // already has the gearing applied, so it cannot tell you the gearing is wrong -- it reports
    // whatever the configured ratio implies, confidently, right up until the pivot hits a stop.
    frc.robot.util.RobotLog.put("IntakePivot/RawMotorRotations",
        pivotMotor.getEncoder().getPosition());
  }

  /**
   * Works out the real pivot gear ratio from a measured sweep between the two hard stops.
   *
   * <p>Needed whenever the gearbox changes. PIVOT_GEARING below converts motor rotations into
   * pivot degrees, and if it is stale every commanded angle is wrong by the same factor. A ratio
   * that is too LARGE makes the pivot overshoot, so it drives past the deployed angle and into
   * the hard stop on every single deploy.
   *
   * <p>Procedure, with the robot DISABLED so nothing drives the pivot while it is moved by hand:
   *
   * <ol>
   *   <li>Put the pivot against the stowed hard stop.</li>
   *   <li>Press the rezero binding to make that position zero.</li>
   *   <li>Move it by hand to the deployed hard stop, a known {@value #PIVOT_MAX_DEGREES} degrees
   *       of travel.</li>
   *   <li>Run this. It prints the ratio that fits what the motor actually turned.</li>
   * </ol>
   */
  public Command printGearingCalibrationCommand() {
    return Commands.runOnce(() -> {
      double rotations = Math.abs(pivotMotor.getEncoder().getPosition());
      double sweptDegrees = PIVOT_MAX_DEGREES - PIVOT_MIN_DEGREES;

      System.out.println("=== INTAKE PIVOT GEARING ===");
      System.out.printf("  motor turned %.3f rotations over a %.0f degree sweep%n",
          rotations, sweptDegrees);

      if (rotations < 0.05) {
        System.out.println("  Too little movement to measure. Rezero at one hard stop, move the");
        System.out.println("  pivot to the other by hand, then run this again.");
        System.out.println("============================");
        return;
      }

      double measured = rotations / (sweptDegrees / 360.0);
      System.out.printf("  measured ratio   %.3f : 1%n", measured);
      System.out.printf("  configured ratio %.3f : 1%n", PIVOT_GEARING);
      System.out.printf("  final stage would be %.3f (currently %.3f)%n",
          measured / (PIVOT_STAGE_1 * PIVOT_STAGE_2), PIVOT_FINAL_STAGE);
      System.out.println("  Put that last number into PIVOT_FINAL_STAGE in IntakeSubsystem.");
      System.out.println("============================");
    }).ignoringDisable(true).withName("IntakePivot.PrintGearingCalibration");
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }

  /**
   * Raises the pivot clear of the shot and holds it there.
   *
   * <p>Held rather than set-and-forget: the pivot has a default command that would otherwise take
   * the subsystem straight back and drop it again mid-shot.
   */
  public Command raiseForShotCommand() {
    return run(() -> intakePivotController.setPosition(Degrees.of(PIVOT_SHOOT_DEGREES)))
        .withName("Intake.raiseForShot");
  }

  /** True once the pivot is high enough to be out of the ball's way. */
  public boolean isClearOfShot() {
    return intakePivot.getAngle().in(Degrees) >= PIVOT_SHOOT_DEGREES - 10.0;
  }
}
