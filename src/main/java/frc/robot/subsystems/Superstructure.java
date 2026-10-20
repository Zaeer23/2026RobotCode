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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.DriveAlignCommand;
import frc.robot.commands.TurretAimCommand;
import frc.robot.vision.Limelight;


/**
 * Superstructure coordinates the shooter, turret and intake subsystems
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
  public final IntakeSubsystem intake;
  public final HopperSubsystem hopper;
  public final KickerSubsystem kicker;
  public final Limelight camera;

  // Tolerance for "at setpoint" checks
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle TURRET_TOLERANCE = Degrees.of(1);

  /** Stick travel past which the operator takes the turret away from vision tracking. */
  private static final double MANUAL_TURRET_DEADBAND = 0.12;

  /** Whether turretAssistCommand currently has the inner tracking command running. */
  private boolean trackingActive = false;

  // Triggers for readiness checks
  private final Trigger isShooterAtSpeed;
  private final Trigger isTurretOnTarget;
  private final Trigger isReadyToShoot;

  /** Readiness while vision aiming, as opposed to holding a manually commanded setpoint. */
  private final Trigger isVisionReadyToShoot;

  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetTurretAngle = Degrees.of(0);

  // Default aim point is red hub

  public Superstructure(ShooterSubsystem shooter, TurretSubsystem turret, IntakeSubsystem intake,
      HopperSubsystem hopper, KickerSubsystem kicker, Limelight camera) {
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;
    this.camera = camera;

    // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
            .in(Degrees));

    // There is no hood on this robot. The shooter fires off a fixed plate at
    // ShooterConstants.LAUNCH_ANGLE_DEGREES, so there is no hood term in readiness at all.
    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget);

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
   * Stops all mechanisms - shooter stops spinning, turret holds position.
   */
  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stop().asProxy(),
        turret.set(0).asProxy()).withName("Superstructure.stopAll");
  }

  /**
   * Aims the superstructure to specific targets - used for auto-targeting.
   *
   * @param shooterSpeed Target shooter speed
   * @param turretAngle  Target turret angle
   */
  public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      targetTurretAngle = turretAngle;
    }).andThen(
        Commands.parallel(
            // shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy()))
        .withName("Superstructure.aim");
  }

  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
  }

  /**
   * Aims the superstructure using suppliers - useful for dynamic targeting.
   *
   * @param shooterSpeedSupplier Supplier for target shooter speed
   * @param turretAngleSupplier  Supplier for target turret angle
   */
  public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy())
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
  public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle) {
    return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle)
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

  /** The fixed launch angle the shooter fires at. There is no hood; this is a plate. */
  public Angle getLaunchAngle() {
    return Degrees.of(ShooterConstants.LAUNCH_ANGLE_DEGREES);
  }

  public AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
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
        Degrees.of(-ShooterConstants.LAUNCH_ANGLE_DEGREES), // pitch is negative launch angle
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

  // ---- Vision -------------------------------------------------------------
  //
  // The whole AprilTag targeting stack was removed and is being rebuilt from the ground up. What
  // exists right now is deliberately only the reading layer: Limelight reports which tags are
  // visible and how far away they are, and nothing steers a mechanism from it yet. Aiming,
  // lining up and vision-driven shooting get layered back on once tag detection is proven on the
  // real robot.

  /**
   * Points the turret at the alliance HUB and keeps it there.
   *
   * <p>See {@link TurretAimCommand} for why the camera's tx is not simply fed to the turret.
   */
  public Command aimTurretCommand() {
    return new TurretAimCommand(turret, camera).withName("Superstructure.aimTurret");
  }

  /**
   * Yaws the chassis to centre on the tag and keeps it centred.
   *
   * <p>Bind to a HELD button. It owns the drivebase for as long as it runs and never finishes by
   * itself, so as a default command the driver would never be able to move.
   */
  public Command alignDriveToTagCommand(SwerveSubsystem drivebase) {
    return new DriveAlignCommand(drivebase, camera).withName("Superstructure.alignDriveToTag");
  }

  /**
   * Range from the shooter exit to the HUB, or -1 when no HUB tag is visible.
   *
   * <p>Ranges off {@code getHubTarget()}, not {@code getTarget()}. The trench is plastered with
   * AprilTags and we shoot from right beside it, so the nearest tag is frequently a TRENCH tag.
   * Ranging off that hands the shot table the distance to the tunnel we are standing next to, and
   * the flywheel spins for a target metres closer than the hub actually is.
   */
  public double getHubDistanceMeters() {
    return camera.getHubTarget().map(tag -> {
      // Measure from where the ball actually leaves, not from the lens.
      //
      // tag.rangeMeters is LENS-to-tag, and the lens is neither at robot centre nor at the
      // turret: it sits 8.5 in to the side and the turret pivot is 0.205 m behind centre. Taking
      // the lens range and subtracting the exit radius mixes two different origins and is wrong
      // by a few inches that change with angle. Going through the robot-frame tag position gets
      // it right at any bearing.
      double pivotX = TurretConstants.PIVOT_TO_ROBOT_CENTER.getX();
      double pivotY = TurretConstants.PIVOT_TO_ROBOT_CENTER.getY();
      double pivotToTag = Math.hypot(tag.robotFrameX() - pivotX, tag.robotFrameY() - pivotY);

      // Once aimed, the ball leaves one exit radius out along the turret's axis, so the shot
      // travels that much less than the pivot-to-tag distance. RANGE_BIAS_METERS then removes the
      // camera's constant over-reporting, which was inflating every shot.
      return Math.max(0.0,
          pivotToTag
              - TurretConstants.SHOOTER_EXIT_RADIUS_METERS
              - ShooterConstants.RANGE_BIAS_METERS);
    }).orElse(-1.0);
  }

  /**
   * True when an actual HUB tag is in view.
   *
   * <p>This was {@code camera.getTarget().isPresent()}, which means "sees any tag at all" — the
   * name was a lie. It is what decides whether an auto takes the vision shot or the blind one, so
   * a trench tag in view was enough to commit to a vision shot ranged off the trench.
   */
  public boolean seesHub() {
    return camera.hasHubTarget();
  }

  /**
   * Spins the flywheel for the currently measured hub range.
   *
   * <p>Aiming is the operator's job for now; this only sets the speed. The turret is not driven
   * from vision until tag detection is trusted.
   */
  /**
   * One-button shot: spin to the range-computed speed, then feed once it is genuinely ready.
   *
   * <p>The feed waits on {@code isAtShotSpeed} rather than firing immediately, because pushing a
   * ball into a flywheel that has not reached speed is the single most reliable way to put it
   * nowhere near the goal. The wait has a timeout so a mis-reported speed cannot lock the shot out
   * entirely — after that it fires anyway and the driver can see the result.
   *
   * <p>Aiming is not part of this. The turret tracks on its own as a default command, so holding
   * this does not fight the driver for the chassis.
   */
  public Command shootWhenReadyCommand() {
    Command spin = shooter.setSpeedForDistance(this::getHubDistanceMeters);

    // The pivot must be clear of the ball's path BEFORE anything feeds. Raising it in parallel
    // with the spin-up costs no time, because the flywheel takes longer to reach speed than the
    // pivot takes to lift.
    Command raisePivot = Constants.IntakeConstants.ENABLED
        ? intake.raiseForShotCommand().asProxy()
        : Commands.none();

    Command fire = Commands.sequence(
        Commands.waitUntil(() ->
            shooter.isAtShotSpeed()
                && (!Constants.IntakeConstants.ENABLED || intake.isClearOfShot()))
            .withTimeout(2.0),
        feedAllCommand());

    // Deadline on `fire`: spin-up and the raised pivot both persist for exactly as long as the
    // shot needs them, then release.
    return Commands.deadline(fire, spin, raisePivot)
        .withName("Superstructure.shootWhenReady");
  }

  public Command shootAtMeasuredRangeCommand() {
    return shooter.setSpeedForDistance(this::getHubDistanceMeters)
        .withName("Superstructure.shootAtMeasuredRange");
  }

  public Command feedAllCommand() {
    if (!Constants.IntakeConstants.ENABLED) {
      return Commands.parallel(
          hopper.feedCommand().asProxy(),
          kicker.feedCommand().asProxy()).withName("Superstructure.feedAll(noIntake)");
    }

    return Commands.parallel(
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy(),
        intake.pivotBounceWhileFeedingCommand().asProxy()).withName("Superstructure.feedAll");
    // intake.setPivotAngle(Degrees.of(46)).asProxy()).withName("Superstructure.feedAll");
  }

  /** Reverses the feed path. Drops the intake leg while the intake is disabled. */
  public Command backFeedAllCommand() {
    if (!Constants.IntakeConstants.ENABLED) {
      return hopper.backFeedCommand().asProxy().withName("Superstructure.backFeedAll(noIntake)");
    }

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

    String readyOut = "R:" + isReadyToShoot.getAsBoolean();

    // System.out.println(shooterOut + " " + turretOut + " " +
    // readyOut);
  }

  public Command useRequirement() {
    return runOnce(() -> {
    });
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation
    // element is based on the fixed launch angle and turret angle
    return new Pose3d(new Translation3d(
        Meter.of(-0.3),
        Meter.of(0),
        Meter.of(0.6)),
        getAimRotation3d());
  }

  public LinearVelocity getTangentialVelocity() {
    return shooter.getTangentialVelocity();
  }
  
  /** True when the camera can see any AprilTag at all. */
  public boolean limelightHasTarget() {
    return !camera.getVisibleTargets().isEmpty();
  }

  /** Horizontal angle to the nearest tag, degrees, POSITIVE to the right. 0 when none visible. */
  public double getNearestTagTxDegrees() {
    return camera.getTarget().map(tag -> tag.txDegrees).orElse(0.0);
  }

  /** Vertical angle to the nearest tag, degrees, POSITIVE up. 0 when none visible. */
  public double getNearestTagTyDegrees() {
    return camera.getTarget().map(tag -> tag.tyDegrees).orElse(0.0);
  }




}
