package frc.robot.subsystems;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems
 * for unified control during shooting operations.
 */
public class Superstructure extends SubsystemBase {

  private static final Set<Integer> CLIMB_BAR_TAG_IDS = Set.of(15, 16, 31, 32);
  private static final Set<Integer> HUB_TAG_IDS = Set.of(9, 10, 11, 18, 19, 20);
  private static final double TURRET_TRACK_TX_FILTER_ALPHA = 0.35;
  private static final double TURRET_TRACK_DEADBAND_DEGREES = 0.75;
  private static final double TURRET_TRACK_OUTPUT_SIGN = -1.0;
  private static final double TURRET_TRACK_KP = 0.018;
  private static final double TURRET_TRACK_MIN_OUTPUT = 0.045;
  private static final double TURRET_TRACK_MAX_OUTPUT = 0.18;

  public final ShooterSubsystem shooter;
  public final TurretSubsystem turret;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  public final HopperSubsystem hopper;
  public final KickerSubsystem kicker;
  public final LimeLight limelight;

  // Tolerance for "at setpoint" checks
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle TURRET_TOLERANCE = Degrees.of(1);
  private static final Angle HOOD_TOLERANCE = Degrees.of(2);

  // Triggers for readiness checks
  private final Trigger isShooterAtSpeed;
  private final Trigger isTurretOnTarget;
  private final Trigger isHoodOnTarget;
  private final Trigger isReadyToShoot;

  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetTurretAngle = Degrees.of(0);
  private Angle targetHoodAngle = Degrees.of(0);

  // Default aim point is red hub

  public Superstructure(ShooterSubsystem shooter, TurretSubsystem turret, HoodSubsystem hood, IntakeSubsystem intake,
      HopperSubsystem hopper, KickerSubsystem kicker, LimeLight limelight) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;
    this.limelight = limelight;

    // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
            .in(Degrees));

    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees));

    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);
  }

  /**
   * Stops all mechanisms - shooter stops spinning, turret and hood hold position.
   */
  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stop().asProxy(),
        turret.set(0).asProxy(),
        hood.set(0).asProxy()).withName("Superstructure.stopAll");
  }

  /**
   * Aims the superstructure to specific targets - used for auto-targeting.
   *
   * @param shooterSpeed Target shooter speed
   * @param turretAngle  Target turret angle
   * @param hoodAngle    Target hood angle
   */
  public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      targetTurretAngle = turretAngle;
      targetHoodAngle = hoodAngle;
    }).andThen(
        Commands.parallel(
            // shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy(),
            hood.setAngle(hoodAngle).asProxy()))
        .withName("Superstructure.aim");
  }

  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
    targetHoodAngle = hoodAngle;
  }

  /**
   * Aims the superstructure using suppliers - useful for dynamic targeting.
   *
   * @param shooterSpeedSupplier Supplier for target shooter speed
   * @param turretAngleSupplier  Supplier for target turret angle
   * @param hoodAngleSupplier    Supplier for target hood angle
   */
  public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy(),
        hood.setAngleDynamic(hoodAngleSupplier).asProxy())
        .withName("Superstructure.aimDynamic");
  }

  // shooting, manual/limelight stuff

  



  /**
   * Waits until the superstructure is ready to shoot.
   */
  public Command waitUntilReadyCommand() {
    return Commands.waitUntil(isReadyToShoot).withName("Superstructure.waitUntilReady");
  }

  /**
   * Aims and waits until ready - combines aim and wait.
   */
  public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle, () -> hoodAngle)
        .andThen(waitUntilReadyCommand())
        .withName("Superstructure.aimAndWait");
  }

  public Command setTurretForward() {
    return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
  }

  public Command setTurretLeft() {
    return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
  }

  public Command setTurretRight() {
    return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
  }

  // Getters for current state
  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Angle getTurretAngle() {
    return turret.getRawAngle();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
  }

  public Angle getTargetHoodAngle() {
    return targetHoodAngle;
  }
// redoing manual code
//  delegates to TurretSubsystem's manualSet which owns the logic because i for some reason am an idiot at coding new commands 🥺
public Command manualTurretControl(Supplier<Double> speedSupplier) {
    return turret.manualSet(speedSupplier).withName("Superstructure.manualTurret");
}
  

  public Rotation3d getAimRotation3d() {
    // See
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Rotation3d(
        Degrees.of(0), // no roll 🤞
        hood.getAngle().unaryMinus(), // pitch is negative hood angle
        turret.getRobotAdjustedAngle());
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.deployAndRollCommand().withName("Superstructure.intake");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.ejectCommand().withName("Superstructure.eject");
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command hopperFeedCommand() {
    return hopper.feedCommand().withName("Superstructure.feed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command hopperReverseCommand() {
    return hopper.reverseCommand().withName("Superstructure.hopperReverse");
  }

  /**
   * Command to run the kicker forward while held, stops when released.
   */
  public Command kickerFeedCommand() {
    return kicker.feedCommand().withName("Superstructure.kickerFeed");
  }

  /**
   * Command to run the kicker stop while held, stops when released.
   */
  public Command kickerStopCommand() {
    return kicker.stopCommand().withName("Superstructure.kickerStop");
  }

  //limelight commands

  public Command limelightAlignCommand(SwerveSubsystem drivebase) {
    return limelight.alignCommand(drivebase).withName("Superstructure.limelightAlign");
  }

  public Command climbBarAlignCommand(SwerveSubsystem drivebase) {
    return limelight.alignToTagsCommand(drivebase, CLIMB_BAR_TAG_IDS)
        .withName("Superstructure.climbBarAlign");
  }

  public Command limelightShootCommand(SwerveSubsystem drivebase) {
    return Commands.parallel(
        shooter.setSpeedFromLimelight(limelight, 45.0, HUB_TAG_IDS),
        trackTargetCommand(limelight))
        .withName("Superstructure.limelightShoot");
  }

  private void runTurretTrackingStep(
      LimeLight limelight,
      double nowSeconds,
      double[] lastLogTimeSecondsHolder,
      double[] filteredTxDegreesHolder,
      boolean[] hasSampleHolder) {
    double lastLogTimeSeconds = lastLogTimeSecondsHolder[0];
    if (lastLogTimeSeconds < 0.0 || nowSeconds - lastLogTimeSeconds >= 0.5) {
      String line = String.format(
          "TURRET_TRACK_HEARTBEAT,time=%.3f,manual_deg=%.2f",
          nowSeconds,
          turret.getRawAngle().in(Degrees));
      System.out.println(line);
      DriverStation.reportWarning(line, false);
      lastLogTimeSecondsHolder[0] = nowSeconds;
    }

    LimeLight.AprilTagScan scan = limelight.scanDirect(HUB_TAG_IDS);
    if (!scan.isValid() || !Double.isFinite(scan.tx)) {
      hasSampleHolder[0] = false;
      turret.setOpenLoop(0.0);
      turret.clearTrackingTelemetry();
      String line = String.format(
          "TURRET_TRACK_DEBUG,NO_TARGET,hasTarget=%b,tag=%d,tx=%.2f",
          scan.hasTarget,
          scan.tagID,
          scan.tx);
      System.out.println(line);
      DriverStation.reportWarning(line, false);
      return;
    }

    double correctedTxDegrees = scan.tx;
    double filteredTxDegrees = filteredTxDegreesHolder[0];
    if (!hasSampleHolder[0]) {
      filteredTxDegrees = correctedTxDegrees;
      hasSampleHolder[0] = true;
    } else {
      filteredTxDegrees +=
          (correctedTxDegrees - filteredTxDegrees) * TURRET_TRACK_TX_FILTER_ALPHA;
    }
    filteredTxDegreesHolder[0] = filteredTxDegrees;

    if (Math.abs(filteredTxDegrees) < TURRET_TRACK_DEADBAND_DEGREES) {
      filteredTxDegrees = 0.0;
      filteredTxDegreesHolder[0] = 0.0;
    }

    double currentTurretDegrees = turret.getRawAngle().in(Degrees);
    double requestedTargetDegrees = currentTurretDegrees - filteredTxDegrees;
    double output = 0.0;
    if (filteredTxDegrees != 0.0) {
      output = TURRET_TRACK_OUTPUT_SIGN * filteredTxDegrees * TURRET_TRACK_KP;
      if (Math.abs(output) < TURRET_TRACK_MIN_OUTPUT) {
        output = Math.copySign(TURRET_TRACK_MIN_OUTPUT, output);
      }
      output = Math.max(-TURRET_TRACK_MAX_OUTPUT, Math.min(TURRET_TRACK_MAX_OUTPUT, output));
    }

    // Never keep driving harder into a mechanical stop.
    if ((currentTurretDegrees <= -89.0 && output < 0.0)
        || (currentTurretDegrees >= 89.0 && output > 0.0)) {
      output = 0.0;
    }

    turret.setOpenLoop(output);
    turret.updateTrackingTelemetry(
        scan.tagID,
        filteredTxDegrees,
        output);
    turret.reportTrackingCommand(
        scan.tagID,
        scan.tx,
        correctedTxDegrees,
        filteredTxDegrees,
        currentTurretDegrees,
        requestedTargetDegrees,
        currentTurretDegrees + output * 100.0);
  }

public Command trackTargetCommand(LimeLight limelight) {
    return Commands.run(
        new Runnable() {
          private final double[] lastLogTimeSeconds = {-1.0};
          private final double[] filteredTxDegrees = {0.0};
          private final boolean[] hasSample = {false};

          @Override
          public void run() {
            double nowSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            runTurretTrackingStep(limelight, nowSeconds, lastLogTimeSeconds, filteredTxDegrees, hasSample);
          }
        },
        turret)
        .beforeStarting(() -> {
          turret.clearTrackingTelemetry();
          System.out.println("TURRET_TRACK_COMMAND_START");
          DriverStation.reportWarning("TURRET_TRACK_COMMAND_START", false);
        })
        .finallyDo(interrupted -> {
          turret.setOpenLoop(0.0);
          turret.clearTrackingTelemetry();
          System.out.println("TURRET_TRACK_COMMAND_END");
          DriverStation.reportWarning("TURRET_TRACK_COMMAND_END", false);
        })
        .withName("Superstructure.trackTarget");
}

  public Command turretAssistCommand(Supplier<Double> manualInputSupplier) {
    return Commands.run(
        new Runnable() {
          private final double[] lastLogTimeSeconds = {-1.0};
          private final double[] filteredTxDegrees = {0.0};
          private final boolean[] hasSample = {false};

          @Override
          public void run() {
          double manualInput = manualInputSupplier.get();
          if (Math.abs(manualInput) > 0.12) {
            double shapedManualInput = Math.copySign(manualInput * manualInput, manualInput);
            turret.setOpenLoop(shapedManualInput);
            turret.clearTrackingTelemetry();
            return;
          }

          runTurretTrackingStep(
              limelight,
              edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
              lastLogTimeSeconds,
              filteredTxDegrees,
              hasSample);
        }},
        turret)
        .finallyDo(interrupted -> {
          turret.setOpenLoop(0.0);
          turret.clearTrackingTelemetry();
        })
        .withName("Superstructure.turretAssist");
  }




  public Command feedAllCommand() {
    return Commands.parallel(
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy(),
        intake.pivotBounceWhileFeedingCommand().asProxy()).withName("Superstructure.feedAll");
    // intake.setPivotAngle(Degrees.of(46)).asProxy()).withName("Superstructure.feedAll");
  }

  public Command backFeedAllCommand() {
    return Commands.parallel(
        hopper.backFeedCommand().asProxy(),
        intake.backFeedAndRollCommand().asProxy()).withName("Superstructure.backFeedAll");
  }

  // public Command intakeBounceCommand() {
  // return Commands.sequence(
  // Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(115))).asProxy()
  // .withName("Superstructure.intakeBounce.deploy"),
  // Commands.waitSeconds(0.5),
  // Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(59))).asProxy()
  // .withName("Superstructure.intakeBounce.feed"),
  // Commands.waitSeconds(0.5))
  // .withName("Superstructure.intakeBounce");
  // }

  public Command stopFeedingAllCommand() {
    return Commands.parallel(
        hopper.stopCommand().asProxy(),
        kicker.stopCommand().asProxy(),
        intake.deployAndRollCommand().asProxy()).withName("Superstructure.stopFeedingAll");
  }

  /**
   * Command to set the intake pivot angle.
   */
  public Command setIntakePivotAngle(Angle angle) {
    return intake.setPivotAngle(angle).withName("Superstructure.setIntakePivotAngle");
  }

  public Command setIntakeDeployAndRoll() {
    return intake.deployAndRollCommand().withName("Superstructure.setIntakeDeployAndRoll");
  }

  /**
   * Command to shoot - spins up shooter.
   */
  public Command shootCommand() {
    // return shooter.sysId().withName("Superstructure.shoot");
    return shooter.spinUp().withName("Superstructure.shoot");
  }

  /**
   * Command to stop shooting - stops shooter.
   */
  public Command stopShootingCommand() {
    return shooter.stop().withName("Superstructure.stopShooting");
  }
  /**
 * Command to set shooter speed dynamically from a supplier.
 * Useful for trigger-scaled or vision-scaled shooting.
 *
 * @param percentSupplier Supplier for target shooter percentage of raw power!!
 */
public Command setShooterPercent(Supplier<Double> percentSupplier) {
    return shooter.setPercent(percentSupplier).withName("Superstructure.setShooterPercent");
}

  // Re-zero intake pivot if needed


  @Override
  public void periodic() {
    // Superstructure doesn't need periodic updates - subsystems handle their own

    String shooterOut = "S:" + isShooterAtSpeed.getAsBoolean() + "(" + Math.round(shooter.getSpeed().in(RPM)) + "/"
        + Math.round(targetShooterSpeed.in(RPM)) + ")";

    String turretOut = "T:" + isTurretOnTarget.getAsBoolean() + "(" + Math.round(turret.getRawAngle().in(Degrees)) + "/"
        + Math.round(targetTurretAngle.in(Degrees)) + ")";

    String hoodOut = "H:" + isHoodOnTarget.getAsBoolean() + "(" + Math.round(hood.getAngle().in(Degrees)) + "/"
        + Math.round(targetHoodAngle.in(Degrees)) + ")";

    String readyOut = "R:" + isReadyToShoot.getAsBoolean();

    // System.out.println(shooterOut + " " + turretOut + " " + hoodOut + " " +
    // readyOut);
  }

  public Command useRequirement() {
    return runOnce(() -> {
    });
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation
    // element is based on hood and turret angles
    return new Pose3d(new Translation3d(
        Meter.of(-0.3),
        Meter.of(0),
        Meter.of(0.6)),
        getAimRotation3d());
  }

  public LinearVelocity getTangentialVelocity() {
    return shooter.getTangentialVelocity();
  }
  
  public boolean limelightHasTarget() {
    return limelight.hasTarget();
  }

  public double getLimeLightApril_HorizontalAngle(LimeLight limelight)
  {
    return limelight.getTX();
  }

  public double getLimeLightApril_VerticalAngle(LimeLight limelight)
  {
    return limelight.getTY();
  }




}
