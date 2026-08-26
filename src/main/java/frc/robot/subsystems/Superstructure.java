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
import frc.robot.FieldConstants;
import frc.robot.commands.HubLineupCommand;
import frc.robot.commands.TurretTrackCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems
 * for unified control during shooting operations.
 */
public class Superstructure extends SubsystemBase {

  // Tag IDs now come from FieldConstants, which reads the official 2026 layout and narrows to the
  // current alliance. HUB_TAG_IDS was Set.of(9, 10, 11, 18, 19, 20): 9/10/11 belong to the RED hub
  // and 18/19/20 to the BLUE hub, so the turret would happily lock onto the opponent's hub, and
  // 10 of the 16 real hub tags were missing entirely.
  //
  // The open-loop tx tracking gains that used to live here moved into TurretTrackCommand, which
  // drives absolute angles through the turret's own position controller instead.

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

  /** Stick travel past which the operator takes the turret away from vision tracking. */
  private static final double MANUAL_TURRET_DEADBAND = 0.12;

  /** Whether turretAssistCommand currently has the inner tracking command running. */
  private boolean trackingActive = false;

  // Triggers for readiness checks
  private final Trigger isShooterAtSpeed;
  private final Trigger isTurretOnTarget;
  private final Trigger isHoodOnTarget;
  private final Trigger isReadyToShoot;

  /** Readiness while vision aiming, as opposed to holding a manually commanded setpoint. */
  private final Trigger isVisionReadyToShoot;

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

    // The hood is not actuated: it is a fixed plate at ShooterConstants.LAUNCH_ANGLE_DEGREES. This
    // compared a hardcoded angle against a target that defaults to 0, so it was permanently false,
    // which made isReadyToShoot permanently false and waitUntilReadyCommand() hang forever.
    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees))
            < HOOD_TOLERANCE.in(Degrees)
            || !hood.isActuated());

    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);

    // While vision aiming, the turret and shooter know their own error against the live target; the
    // fields above only track manually commanded setpoints and would report a stale "ready".
    this.isVisionReadyToShoot = new Trigger(
        () -> turret.isOnTarget() && shooter.isAtShotSpeed());
  }

  /** True when the turret is on the tag and the flywheel is at the vision-computed speed. */
  public boolean isVisionReadyToShoot() {
    return isVisionReadyToShoot.getAsBoolean();
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

  /**
   * Lines up on the TOWER tags for climbing.
   *
   * <p>Deferred so the alliance-correct tag set resolves when the command is scheduled, not when it
   * is constructed at boot — the driver station has not reported an alliance yet at that point.
   */
  public Command climbBarAlignCommand(SwerveSubsystem drivebase) {
    return Commands.defer(
        () -> limelight.alignToTagsCommand(drivebase, FieldConstants.towerTagIds()),
        Set.of(drivebase))
        .withName("Superstructure.climbBarAlign");
  }

  /**
   * Drives the chassis to the calibrated shooting spot on the hub.
   *
   * <p>Bind to a HELD button. It owns the drivebase while it runs, so as a default command it would
   * leave the driver unable to move.
   */
  public Command hubLineupCommand(SwerveSubsystem drivebase) {
    return new HubLineupCommand(drivebase, limelight).withName("Superstructure.hubLineup");
  }

  /**
   * Aims and spins from vision without moving the chassis, so the driver keeps the drivebase and
   * fires manually.
   */
  public Command limelightShootCommand(SwerveSubsystem drivebase) {
    return Commands.parallel(
        shooter.setSpeedFromLimelight(limelight, drivebase),
        trackHubTagsCommand(drivebase))
        .withName("Superstructure.limelightShoot");
  }

  /**
   * Full one-button shot: drive to range, hold the turret on the hub, spin the flywheel to the
   * vision-computed speed, and feed once both are genuinely ready.
   *
   * <p>The lineup runs only until it settles and then releases the chassis, so holding the button
   * does not pin the robot in place for the rest of the match.
   */
  public Command autoShootCommand(SwerveSubsystem drivebase) {
    Command aimAndSpin = Commands.parallel(
        trackHubTagsCommand(drivebase),
        shooter.setSpeedFromLimelight(limelight, drivebase));

    Command lineupThenFire = Commands.sequence(
        new HubLineupCommand(drivebase, limelight),
        Commands.waitUntil(isVisionReadyToShoot).withTimeout(1.5),
        feedAllCommand());

    return Commands.deadline(lineupThenFire, aimAndSpin)
        .withName("Superstructure.autoShoot");
  }

  /**
   * Closed-loop turret tracking on the alliance HUB.
   *
   * <p>The implementation moved to {@link TurretTrackCommand}. The old version lived here as a
   * hand-rolled open-loop tx chase with a minimum-output floor, which hunted around the target and
   * derived its setpoint from the turret's own position. See that class for the details.
   */
  public Command trackTargetCommand(LimeLight limelight) {
    return new TurretTrackCommand(turret, limelight, null).withName("Superstructure.trackTarget");
  }

  /** Closed-loop turret tracking with chassis yaw-rate latency compensation. */
  public Command trackHubTagsCommand(SwerveSubsystem drivebase) {
    return new TurretTrackCommand(turret, limelight, drivebase)
        .withName("Superstructure.trackHubTags");
  }

  /**
   * Vision tracking that yields to the operator: whenever the manual stick is pushed past its
   * deadband the turret follows the stick, and it returns to tracking the moment they let go.
   */
  public Command turretAssistCommand(Supplier<Double> manualInputSupplier) {
    Command track = new TurretTrackCommand(turret, limelight, null);
    return Commands.run(
        () -> {
          double manualInput = manualInputSupplier.get();
          if (Math.abs(manualInput) > MANUAL_TURRET_DEADBAND) {
            if (trackingActive) {
              track.end(true);
              trackingActive = false;
            }
            turret.setOpenLoop(Math.copySign(manualInput * manualInput, manualInput));
            return;
          }

          if (!trackingActive) {
            track.initialize();
            trackingActive = true;
          }
          track.execute();
        },
        turret)
        .finallyDo(interrupted -> {
          if (trackingActive) {
            track.end(true);
            trackingActive = false;
          }
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

  /**
   * Stops the feed path.
   *
   * <p>The intake is deliberately left alone rather than commanded. This used to end by calling
   * {@code intake.deployAndRollCommand()}, so "stop feeding" actually deployed the intake and spun
   * its rollers up — the opposite of stopping, and a nasty surprise at the end of an autonomous
   * routine.
   */
  public Command stopFeedingAllCommand() {
    return Commands.parallel(
        hopper.stopCommand().asProxy(),
        kicker.stopCommand().asProxy()).withName("Superstructure.stopFeedingAll");
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
